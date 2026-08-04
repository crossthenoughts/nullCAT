// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// SdoWorker.cpp   (see SdoWorker.h for the full contract)
//
// SDO is issued via SOEM's native cyclic-mailbox path (ecx_SDOread/ecx_SDOwrite). In
// OP the slaves are ECT_MBXH_CYCLIC, so those calls enqueue and BLOCK ON THIS non-RT
// THREAD until the RT loop's already-running processCyclicMailbox() (limit=1, ≤1
// mailbox datagram/cycle) services them. No soemAccessMutex here (would deadlock the
// handler we wait on); whole-transfer cross-actor exclusion is sdoTransferMutex.
// ============================================================
#include "SdoWorker.h"
#include "EtherCATMaster.h"
#include "Logging.h"
#include "PlatformRT.h"   // safeCall — SEH/signal guard around SOEM calls

#ifdef SOEM_AVAILABLE
extern "C" {
#include "soem/ec_options.h"
#include "soem/ec_type.h"
#include "nicdrv.h"
#include "soem/ec_base.h"
#include "soem/ec_main.h"
#include "soem/ec_coe.h"
}
#endif

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>

// ---- monotonic seconds (steady clock) ----
static double nowSec()
{
    using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now().time_since_epoch()).count();
}

SdoWorker::SdoWorker(EtherCATMaster& master) : m_master(master) {}
SdoWorker::~SdoWorker() { stop(); }

void SdoWorker::start()
{
    if (m_thread.joinable()) return;
    m_stop.store(false, std::memory_order_release);
    m_thread = std::thread(&SdoWorker::run, this);
}

void SdoWorker::stop()
{
    if (!m_thread.joinable()) return;
    m_stop.store(true, std::memory_order_release);
    m_cv.notify_all();
    m_thread.join();
}

uint64_t SdoWorker::submitRead(uint16_t slave, uint16_t index, uint8_t sub, uint8_t size)
{
    SdoRequest r; r.op = SdoRequest::Op::Read; r.slave = slave; r.index = index;
    r.sub = sub; r.size = size; r.id = m_nextId.fetch_add(1, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lk(m_mu);
        m_intake.push_back(r);
        m_results.push_back(SdoResult{ r.id, SdoResult::Status::Pending, 0, 0, 0 });
    }
    m_cv.notify_one();
    return r.id;
}

uint64_t SdoWorker::submitWrite(uint16_t slave, uint16_t index, uint8_t sub, uint8_t size, uint32_t value)
{
    SdoRequest r; r.op = SdoRequest::Op::Write; r.slave = slave; r.index = index;
    r.sub = sub; r.size = size; r.value = value;
    r.id = m_nextId.fetch_add(1, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lk(m_mu);
        m_intake.push_back(r);
        m_results.push_back(SdoResult{ r.id, SdoResult::Status::Pending, 0, 0, 0 });
    }
    m_cv.notify_one();
    return r.id;
}

bool SdoWorker::poll(uint64_t id, SdoResult& out)
{
    std::lock_guard<std::mutex> lk(m_mu);
    for (size_t i = 0; i < m_results.size(); ++i)
    {
        if (m_results[i].id != id) continue;
        out = m_results[i];
        // retire a terminal result so the vector stays small
        if (out.status != SdoResult::Status::Pending)
            m_results.erase(m_results.begin() + i);
        return true;
    }
    return false;
}

void SdoWorker::configureTempPoll(uint16_t index, uint8_t sub, uint8_t size,
                                  double scaleToC, double intervalSecPerDrive, int driveCount)
{
    std::lock_guard<std::mutex> lk(m_mu);
    m_temp.index = index; m_temp.sub = sub; m_temp.size = size;
    m_temp.scaleToC = scaleToC; m_temp.intervalSec = (intervalSecPerDrive > 0.5) ? intervalSecPerDrive : 0.5;
    m_temp.driveCount = driveCount; m_temp.enabled = (driveCount > 0 && index != 0);
    m_tempC.assign(driveCount + 1, 0.0);
    m_tempValid.assign(driveCount + 1, false);
    m_tempNextDue.assign(driveCount + 1, 0.0);   // all due immediately
    m_cv.notify_one();
}

bool SdoWorker::tempValid(uint16_t slave) const
{
    std::lock_guard<std::mutex> lk(m_mu);
    return slave < m_tempValid.size() && m_tempValid[slave];
}

double SdoWorker::tempLatestC(uint16_t slave) const
{
    std::lock_guard<std::mutex> lk(m_mu);
    return (slave < m_tempC.size()) ? m_tempC[slave] : 0.0;
}

void SdoWorker::onChainReinit()
{
    m_reinitGen.fetch_add(1, std::memory_order_acq_rel);   // invalidate any in-flight op
    std::lock_guard<std::mutex> lk(m_mu);
    m_intake.clear();
    for (auto& r : m_results) if (r.status == SdoResult::Status::Pending) r.status = SdoResult::Status::Discarded;
    std::fill(m_tempValid.begin(), m_tempValid.end(), false);
    std::fill(m_tempNextDue.begin(), m_tempNextDue.end(), 0.0);
}

// ============================================================
// One expedited CoE transfer, via SOEM's NATIVE cyclic-mailbox SDO path.
//
// In OP the slaves are in ECT_MBXH_CYCLIC. ecx_SDOread/ecx_SDOwrite then enqueue the
// request into SOEM's internally-mutexed mailbox queue and BLOCK ON THIS (non-RT)
// THREAD until the RT loop's processCyclicMailbox() services them — bounded to <=1
// mailbox datagram per cycle (that drain is already unconditional in ControlLoop, the
// limit=1 path designed for "queued SDOs"). So the worker adds NO new per-cycle work
// to the RT loop and never raises its <=1-op/cycle ceiling; it only gives the existing
// handler an op to service more often. Rig-validated to stay jitter/DC-neutral.
//
// Therefore: do NOT hold soemAccessMutex here (the RT handler we are waiting on needs
// it -> deadlock) and do NOT suspend the handler (it is the courier that both transmits
// the queued request and delivers the response). Cross-actor whole-transfer exclusion
// (vs recovery / provisioning) is the caller's sdoTransferMutex. safeCall guards the
// SOEM call as everywhere else in this codebase.
// ============================================================
#ifdef SOEM_AVAILABLE
SdoResult SdoWorker::execTransfer(const SdoRequest& req, void* ctxv)
{
    ecx_contextt* ctx = static_cast<ecx_contextt*>(ctxv);
    SdoResult res; res.id = req.id; res.status = SdoResult::Status::Failed;

    const int timeoutUs = 700000;   // 700 ms, matching the other CoE SDO calls
    uint32_t exCode = 0;
    int wkc = 0;

    if (req.op == SdoRequest::Op::Read)
    {
        int psize = (req.size >= 1 && req.size <= 4) ? req.size : 4;   // expedited cap
        uint8_t buf[4] = { 0, 0, 0, 0 };
        PlatformRT::safeCall([&]() {
            wkc = ecx_SDOread(ctx, req.slave, req.index, req.sub, FALSE, &psize, buf, timeoutUs);
        }, &exCode);
        res.wkc = wkc;
        if (exCode == 0 && wkc > 0)
        {
            uint32_t v = 0;
            const int n = (psize >= 1 && psize <= 4) ? psize : 0;   // bytes actually returned
            for (int i = 0; i < n; ++i) v |= static_cast<uint32_t>(buf[i]) << (8 * i);
            res.value = v;
            res.status = SdoResult::Status::Done;
        }
    }
    else
    {
        uint32_t val = req.value;   // low `size` bytes are sent, little-endian
        PlatformRT::safeCall([&]() {
            wkc = ecx_SDOwrite(ctx, req.slave, req.index, req.sub, FALSE,
                               static_cast<int>(req.size), &val, timeoutUs);
        }, &exCode);
        res.wkc = wkc;
        if (exCode == 0 && wkc > 0) res.status = SdoResult::Status::Done;
    }
    // wkc<=0 stays Failed. A CoE abort surfaces as wkc<=0 plus an entry in SOEM's error
    // list; surfacing the precise abort code (drainElist) is a future refinement.
    return res;
}
#else
SdoResult SdoWorker::execTransfer(const SdoRequest&, void*) { return {}; }
#endif

// ============================================================
// Worker thread: gate on stably-OP, run one transfer at a time under sdoTransferMutex
// (cross-actor whole-transfer exclusion), publish results, schedule the temp round-robin.
// ============================================================
void SdoWorker::run()
{
    while (!m_stop.load(std::memory_order_acquire))
    {
        SdoRequest req; bool haveWork = false; bool isTemp = false; uint16_t tempSlave = 0;

        // ---- pick work: a due temp read, else a submitted op ----
        {
            std::unique_lock<std::mutex> lk(m_mu);
            const double t = nowSec();
            if (m_temp.enabled)
            {
                for (int s = 1; s <= m_temp.driveCount; ++s)
                    if (m_tempNextDue[s] <= t)
                    {
                        req = SdoRequest{}; req.op = SdoRequest::Op::Read; req.slave = (uint16_t)s;
                        req.index = m_temp.index; req.sub = m_temp.sub; req.size = m_temp.size;
                        haveWork = true; isTemp = true; tempSlave = (uint16_t)s;
                        break;
                    }
            }
            if (!haveWork && !m_intake.empty()) { req = m_intake.front(); m_intake.pop_front(); haveWork = true; }
            if (!haveWork)
            {
                // sleep until the soonest temp-due time (or 1s), woken by submit/reinit/stop
                double wait = 1.0;
                if (m_temp.enabled)
                    for (int s = 1; s <= m_temp.driveCount; ++s)
                        wait = std::min(wait, std::max(0.0, m_tempNextDue[s] - t));
                m_cv.wait_for(lk, std::chrono::duration<double>(wait > 0 ? wait : 0.001));
                continue;
            }
        }

        // ---- state gate: only attempt SDO when stably in OP AND the RT loop is active ----
        // The worker's SDO is serviced by processCyclicMailbox(), which ONLY the control loop
        // drives. While the background pump holds OP (loop stopped), the mailbox isn't serviced,
        // so a transfer would just time out -- so HOLD (don't drop) until the loop runs. (PREOP
        // provisioning is a separate direct-SDO path; it never goes through this worker.)
        if (!(m_master.isOperational() && !m_master.isInitializing() && m_master.isRtLoopActive()))
        {
            if (!isTemp) { std::lock_guard<std::mutex> lk(m_mu); m_intake.push_front(req); }  // re-queue one-offs
            std::this_thread::sleep_for(std::chrono::milliseconds(20));   // wait for loop-active OP
            continue;
        }

        // ---- whole-transfer ownership + live ctx (re-init safe) ----
        const uint64_t gen = m_reinitGen.load(std::memory_order_acquire);
        SdoResult result; result.id = req.id; result.status = SdoResult::Status::Failed;
        {
            std::lock_guard<std::mutex> xfer(m_master.sdoTransferMutex());  // cross-actor exclusion
            // re-check after acquiring the transfer lock (shutdown holds it before freeing ctx)
            if (m_master.isOperational() && !m_master.isInitializing() && m_master.isRtLoopActive()
                && m_reinitGen.load(std::memory_order_acquire) == gen)
            {
                void* ctx = m_master.getContext();
                if (ctx)
                {
                    // Hint the RT loop that mailbox work is in flight so
                    // processCyclicMailbox runs at full rate for the duration
                    // (it idles at 1/16 duty otherwise -- see EtherCATMaster).
                    m_master.beginMailboxWork();
                    result = execTransfer(req, ctx);   // blocks on THIS thread; RT handler services it
                    m_master.endMailboxWork();
                }
            }
        }

        // ---- publish, unless a re-init happened during the transfer (discard stale) ----
        if (m_reinitGen.load(std::memory_order_acquire) != gen) continue;
        double tempC = 0.0;
        {
            std::lock_guard<std::mutex> lk(m_mu);
            if (isTemp)
            {
                tempC = result.value * m_temp.scaleToC;
                if (result.status == SdoResult::Status::Done && tempSlave < m_tempC.size())
                {
                    m_tempC[tempSlave] = tempC;
                    m_tempValid[tempSlave] = true;
                }
                if (tempSlave < m_tempNextDue.size())
                    m_tempNextDue[tempSlave] = nowSec() + m_temp.intervalSec;   // reschedule regardless
            }
            else
            {
                for (auto& r : m_results) if (r.id == req.id) { r = result; break; }
            }
        }
        // Log temp reads (non-RT thread -> LOG_* is fine) so the read is verifiable on the
        // rig. Off the m_mu lock. Each successful round-robin read prints the value.
        if (isTemp)
        {
            if (result.status == SdoResult::Status::Done)
                LOG_INFO(strf("SdoWorker: drive %u IGBT temp = %.1f C", (unsigned)tempSlave, tempC));
            else
                LOG_WARNING(strf("SdoWorker: drive %u temp read failed (status=%d wkc=%d abort=0x%08x)",
                                 (unsigned)tempSlave, (int)result.status, result.wkc, result.abortCode));
        }
    }
}
