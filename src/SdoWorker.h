// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// SdoWorker.h
//
// A non-RT, LOW-VOLUME CoE SDO worker. During OP its only ongoing job is a slow
// round-robin temperature poll (~1 SDO/s even at 10 drives); live writes and
// inertia-ID are occasional. Engineered for CORRECTNESS, not throughput - // single-serial.
//
// HOW IT STAYS OFF THE RT CRITICAL PATH (SOEM's native cyclic mailbox):
//   During OP the slaves are in ECT_MBXH_CYCLIC. In that mode ecx_SDOread/
//   ecx_SDOwrite do NOT touch the wire directly - they enqueue the request into
//   SOEM's internally-mutexed mailbox queue and BLOCK ON THE CALLING (worker)
//   THREAD until the RT loop's processCyclicMailbox() services them. That cyclic
//   drain is already unconditional in ControlLoop with limit=1 (≤1 mailbox datagram
//   per cycle - the path designed for "queued SDOs"). So this worker adds NO new
//   per-cycle work to the RT loop and never raises its ≤1-op/cycle ceiling; it only
//   gives the existing handler an op to service more often. The blocking wait lives
//   entirely on this non-RT thread. (Rig-validated: servicing temp polls at the
//   bounded rate leaves jitter/DC-phase/WKC flat - SOEM's bounded cyclic cost is
//   negligible in this loop.)
//
//   CONSEQUENCE: the worker must NOT hold soemAccessMutex across an SDO call (the RT
//   handler it is waiting on needs that mutex → deadlock), and must NOT suspend the
//   handler (it is the courier that transmits the request AND delivers the response,
//   demuxing EMCY → emergency handler vs SDO → coembxin by type).
//
// CONCURRENCY CONTRACT (one lock):
//   * sdoTransferMutex (WHOLE transfer): held for an entire transfer by ANY SDO actor
//     (this worker, the recovery thread's fault-history read, PREOP provisioning).
//     Guarantees (a) one CoE transaction per slave at a time across actors - no
//     interleave, and (b) the context is not freed mid-transfer (shutdown/re-init
//     stop-join the worker before freeing m_ctx). The RT loop's per-step access is its
//     own soemAccessMutex (the worker never takes it), so there is no lock-order cycle.
//
// STATE GATE (no TOCTOU): a transfer only STARTS when stably in OP. The "stably-OP?"
// check and the ctx fetch happen together under sdoTransferMutex; queued ops are HELD
// across transitions, never dropped.
//
// RE-INIT: the worker is stop()-joined BEFORE m_ctx is freed/realloced (in
// initializeAndEnterOp and shutdown), so no transfer can touch a stale context;
// onChainReinit() discards queued + in-flight ops and cached temps.
//
// SCOPE: EXPEDITED SDO only (≤4 bytes) - covers temp, 0x6065, opMode, all
// immediate params. Segmented transfer (strings / OD lists) is a clean follow-up.
// Provisioning's heavy batch SDO is a SEPARATE PREOP path (classic mailbox, no RT
// loop), not this worker.
//
// WRITES: encoding is SOEM's own ecx_SDOwrite (no hand-rolled codec to mis-encode).
// readback-verify ("did it take") remains the FIRST write-caller's responsibility - // no write path is active yet (temp poll is read-only; submitWrite is unused),
// so when provisioning/opMode/live-param writes are added, they MUST readback-verify
// each write to a safety object (0x6072 torque limit, 0x6065 FE window).
// ============================================================

#include <cstdint>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <vector>

class EtherCATMaster;   // owns ctx, soemAccessMutex, sdoTransferMutex, master state

// One queued SDO operation. Expedited only (size 1..4).
struct SdoRequest
{
    enum class Op : uint8_t { Read, Write };
    Op       op    = Op::Read;
    uint16_t slave = 0;     // 1-based SOEM slave index
    uint16_t index = 0;
    uint8_t  sub   = 0;
    uint8_t  size  = 0;     // 1..4 bytes (expedited)
    uint32_t value = 0;     // Write: value in (low `size` bytes, little-endian)
    uint64_t id    = 0;     // assigned by submit*(); poll() by this id
};

struct SdoResult
{
    enum class Status : uint8_t { Pending, Done, Failed, Aborted, Timeout, Discarded };
    uint64_t id        = 0;
    Status   status    = Status::Pending;
    uint32_t value     = 0;   // Read: value out
    uint32_t abortCode = 0;   // CoE abort code when status == Aborted
    int      wkc       = 0;
};

class SdoWorker
{
public:
    explicit SdoWorker(EtherCATMaster& master);
    ~SdoWorker();

    SdoWorker(const SdoWorker&) = delete;
    SdoWorker& operator=(const SdoWorker&) = delete;

    // Thread lifecycle. stop() is stop-first: signals, lets any in-flight transfer
    // finish/abort under the transfer mutex, joins. Safe to call before ecx_close.
    void start();
    void stop();

    // ---- generic one-off ops (live WebUI writes, inertia-ID trigger, OP-time reads) ----
    // Non-blocking submit (mutex-guarded intake; many producers OK). Returns an id.
    // The op executes only once stably in OP; poll(id) for the result.
    uint64_t submitRead (uint16_t slave, uint16_t index, uint8_t sub, uint8_t size);
    uint64_t submitWrite(uint16_t slave, uint16_t index, uint8_t sub, uint8_t size, uint32_t value);
    // Copies the current result for id. When the result is terminal it is retired
    // (one successful terminal poll frees the slot). Returns false if id unknown.
    bool poll(uint64_t id, SdoResult& out);

    // ---- temperature round-robin (the only ongoing OP-time SDO) ----
    // Object + scaling are injected (not hardcoded) - default target is the IGBT
    // power-stage sensor (0x2040:0x31, U16, 0.1 C) that backs Er42.2. intervalSec is
    // PER DRIVE (default 15s -> ~1 op/s at 10 drives). driveCount is slaves 1..N.
    void configureTempPoll(uint16_t index, uint8_t sub, uint8_t size,
                           double scaleToC, double intervalSecPerDrive, int driveCount);
    bool   tempValid(uint16_t slave) const;   // slave is 1-based
    double tempLatestC(uint16_t slave) const;

    // Called by EtherCATMaster on a chain drop / re-init: discard ALL queued and
    // in-flight ops and cached temps so nothing stale is applied to the new context.
    void onChainReinit();

private:
    void run();                       // worker thread body
    // One expedited transfer via SOEM's native cyclic SDO (ecx_SDOread/write). ctx is
    // the live ecx_contextt* (void* keeps SOEM out of this header); caller holds
    // sdoTransferMutex. Blocks on the worker thread; the RT loop's cyclic handler
    // services it ≤1 op/cycle. NO soemAccessMutex (would deadlock the handler).
    SdoResult execTransfer(const SdoRequest& req, void* ctx);

    EtherCATMaster& m_master;

    std::thread              m_thread;
    std::atomic<bool>        m_stop{false};
    mutable std::mutex       m_mu;    // guards intake, results, temp cache, sched
    std::condition_variable  m_cv;

    std::deque<SdoRequest>   m_intake;     // submitted, not yet started
    std::vector<SdoResult>   m_results;    // terminal results retired on poll
    std::atomic<uint64_t>    m_nextId{1};
    std::atomic<uint64_t>    m_reinitGen{0};  // bumped on onChainReinit(); tags in-flight op

    // temp round-robin
    struct TempCfg { uint16_t index=0; uint8_t sub=0; uint8_t size=2; double scaleToC=0.1;
                     double intervalSec=15.0; int driveCount=0; bool enabled=false; };
    TempCfg                  m_temp;
    std::vector<double>      m_tempC;       // latest, per 1-based slave (index 0 unused)
    std::vector<bool>        m_tempValid;
    std::vector<double>      m_tempNextDue; // monotonic-seconds due time per slave
};
