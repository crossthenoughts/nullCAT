// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// EtherCATMaster.cpp - SOEM-based EtherCAT master. Owns bus init
// (PreOp → SafeOp → OP with DC/SYNC0 arming), cyclic process-data
// exchange, the background pump, the recovery thread, and bus
// diagnostics.
//
// Platform-specific RT primitives (SEH wrappers, high-resolution
// timing, timer period control) are abstracted behind PlatformRT.h.
// Adapter matching falls back to the raw nicName if no pcap match
// is found.
// ============================================================

#include "EtherCATMaster.h"
#include "SdoWorker.h"
#include "Logging.h"
#include "PlatformRT.h"
#include "DcPhaseLock.h"

extern "C" {
#include "soem/ec_options.h"
#include "soem/ec_type.h"
#include "nicdrv.h"
#include "soem/ec_base.h"
#include "soem/ec_main.h"
#include "soem/ec_dc.h"
#include "soem/ec_coe.h"
#include "soem/ec_config.h"
#include "soem/ec_print.h"   // ec_ALstatuscode2string, ecx_err2string, ecx_elist2string
}

#include <chrono>
#include <cstring>

#ifndef EC_TIMEOUTSTATE
#  define EC_TIMEOUTSTATE  2000000
#endif
#ifndef EC_TIMEOUTRET
#  define EC_TIMEOUTRET    2000
#endif

static ecx_contextt* ctxPtr(void* p) { return static_cast<ecx_contextt*>(p); }

// Shared state for the PO2SO hook. Set by tryInitOnceBody() before hook
// registration; read by dcsyncHook() during ecx_config_map_group(). Safe because
// EC_MAX_MAPT=1 serialises slave processing and init runs on a single thread.
static uint32_t s_dcSyncNs       = 0;
static int32_t  s_dcSyncOffsetNs = 0;

static int dcsyncHook(ecx_contextt* context, uint16 slave)
{
    ecx_dcsync0(context, slave, TRUE, s_dcSyncNs, s_dcSyncOffsetNs);
    return 1;
}

// Drain any queued emergencies from a slave's outbound mailbox (SM1) before
// the first SDO exchange. ArthurKetels (SOEM issue #951): dirty drives may have
// accumulated emergencies that block SDO operations; drain time "can range from
// ms to seconds depending on the slave".
//
// Returns the total number of messages consumed (0 = mailbox was clean).
// Emergencies are auto-handled by ecx_mbxreceive into ctx->elist; call
// drainElist() after to log them with full error code details.
static int drainSlaveMailbox(ecx_contextt* ctx, int slave)
{
    // ArthurKetels: allow "ms to seconds". 500ms covers burst residue without
    // stalling a clean init.
    const int kDrainTimeoutUs = 500000;

    auto t0 = std::chrono::steady_clock::now();
    int count = 0;

    for (;;)
    {
        int elapsedUs = (int)std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t0).count();
        if (elapsedUs >= kDrainTimeoutUs) break;

        ec_mbxbuft* mbx = nullptr;
        int wkc = ecx_mbxreceive(ctx, slave, &mbx, 0);  // timeout=0: non-blocking SM1 poll

        // wkc == EC_TIMEOUT (-5): SM1 bit 3 = 0, mailbox is empty - drain complete.
        // wkc == 0: ecx_mbxreceive processed an emergency internally (pushed to
        //   ctx->elist, buffer already dropped) - keep draining, more may follow.
        // These two cases MUST be distinguished. Treating wkc<=0 as "done" would
        // stop the drain on the first emergency instead of continuing to flush.
        if (wkc == EC_TIMEOUT) break;

        ++count;
        if (mbx)
        {
            // Unexpected non-emergency message (CoE response, EoE, etc.) queued
            // before any SDO request - log mbxtype and discard.
            uint8_t mbxtype = reinterpret_cast<uint8_t*>(mbx)[4] & 0x0f;
            LOG_WARNING(strf("  Slave %d: drain discarded unexpected mbxtype=0x%02x", slave, mbxtype));
            ecx_dropmbx(ctx, mbx);
        }

        osal_usleep(1000);  // 1ms between polls - prevents tight spin on multi-message burst
    }

    return count;
}

EtherCATMaster::EtherCATMaster()
{
    memset(m_iomap, 0, sizeof(m_iomap));
    startPumpDispatchThread();
    // Own the non-RT SDO worker. The thread is created lazily by
    // start() (called once OP is reached); until then this just allocates the object.
    m_sdoWorker = std::make_unique<SdoWorker>(*this);
}

// Destructor leaves m_ctx pointing at a fresh zeroed context so any
// late reader sees inert state rather than freed memory.
EtherCATMaster::~EtherCATMaster()
{
    if (m_initialized)
        shutdown();
    // Stop dispatch thread if not already stopped by shutdown()
    if (m_pumpDispatchThread.joinable())
    {
        {
            std::lock_guard<std::mutex> lk(m_pumpDispatchMu);
            m_pumpDispatchStop = true;
        }
        m_pumpDispatchCv.notify_one();
        m_pumpDispatchThread.join();
    }
    delete ctxPtr(m_ctx);
    ecx_contextt* ctx = new ecx_contextt();
    memset(ctx, 0, sizeof(ecx_contextt));
    m_ctx = ctx;
}

void EtherCATMaster::setDriveConfigs(const std::vector<DriveConfig>& configs)
{
    m_driveConfigs = configs;
}

void EtherCATMaster::applyConfig(const AppConfig& cfg)
{
    setDriveConfigs(cfg.drives);
    setControlLoopHz(cfg.controlLoopHz);
    setDcSyncOffsetNs(cfg.dcSyncOffsetNs);
    setDcPhaseLock(cfg.dcPhaseLockEnabled, cfg.dcPhaseLockKp,
                   cfg.dcPhaseLockKi, cfg.dcPhaseLockMaxTrimNs);   // default OFF
    setPdoWatchdogMs(cfg.pdoWatchdogMs);
    setTempPollSec(cfg.tempPollSec);   // IGBT temp round-robin (0 = off)
    setEnableCapabilityScan(cfg.enableCapabilityScan);
    m_commandSyncCycles      = cfg.commandSyncCycles;
    m_wkcValidationCycles    = cfg.wkcValidationCycles;
    m_wkcValidationThreshold = cfg.wkcValidationThreshold;
}

InitResult EtherCATMaster::initializeAndEnterOp(const std::string& nicName)
{
    // Prevent concurrent calls from Qt button and web API
    bool expected = false;
    if (!m_initInProgress.compare_exchange_strong(expected, true))
    {
        m_lastError = "Initialization already in progress.";
        LOG_WARNING("EtherCATMaster: " + m_lastError);
        return InitResult::fail(InitError::AlreadyInProgress, m_lastError);
    }

    // RAII guard: always clear m_initInProgress on exit
    struct Guard {
        std::atomic<bool>& flag;
        ~Guard() { flag.store(false); }
    } guard{m_initInProgress};

    // Quiesce the SDO worker before (re-)init. stop() JOINS it, so no
    // worker transfer can be touching m_ctx while initialize() frees/reallocs it
    // (stronger than a generation check). onChainReinit() drops any stale queued ops
    // and cached temps. Idempotent if the worker was never started.
    if (m_sdoWorker) { m_sdoWorker->stop(); m_sdoWorker->onChainReinit(); }

    { auto r = initialize(nicName);   if (!r.ok) return r; }
    { auto r = configurePDOs();       if (!r.ok) return r; }
    { auto r = enterOperational();    if (!r.ok) return r; }

    // Start the canonical recovery thread once OP is reached. The
    // thread idles on a 10ms tick until signalRecoveryNeeded() is called by
    // ControlLoopWorker on a sustained WKC error.
    startRecoveryThread();

    // Start the SDO worker now that the chain is OP and the cyclic
    // mailbox handler is registered. Temp poll is enabled only if configured > 0
    // (default off → worker idles, no bus traffic). 0x2040:0x31 is the AS715N IGBT
    // power-stage sensor (U16, 0.1 C) backing Er42.2; interval is per-drive.
    if (m_sdoWorker && !m_simulationMode)
    {
        if (m_tempPollSec > 0.0)
        {
            m_sdoWorker->configureTempPoll(0x2040, 0x31, 2, 0.1, m_tempPollSec,
                                           static_cast<int>(m_drives.size()));
            LOG_INFO(strf("EtherCATMaster: SDO worker started (temp poll every %.0fs/drive).",
                          m_tempPollSec));
        }
        else
        {
            LOG_INFO("EtherCATMaster: SDO worker started (temp poll disabled - no bus traffic).");
        }
        m_sdoWorker->start();
    }
    return InitResult::success();
}

// ============================================================
// Diagnostic instrumentation.
//
// All three functions write to the separate _soem.log file via
// Logger::logDiag() using pipe-delimited key=value format:
//   <timestamp> | DIAG | <category> | location=X | key=val | ...
//
// Valid location strings (fixed vocabulary):
//   pre_mbx_ping - after BRD flush, before first SDO
//   post_config_map - after ecx_config_map_group()
//   post_safeop_sdos - after SafeOp mode/torque/following-error SDOs
//   pre_config_map - immediately before ecx_config_map_group(), every attempt
//   pre_op_request - after pre-OP pump, before writing OP state
//   post_op_result - after ecx_readstate(), success or failure
//   pre_op_pump - periodic sample during 4s pre-OP pump (cycle= field)
//   op_pump - periodic sample during 5s OP pump (cycle= field)
//
// drainElist:    reads and clears SOEM's internal error ring (ctx->elist).
//                total=0 | status=empty is the expected outcome on a clean
//                session. EMERGENCY entries on dirty drives are normal (drive
//                queued them before the previous unclean close). SDO aborts
//                in other etype values may indicate config mapping failures.
//
// dumpMbxState:  reads SM1 status register (0x080D) per slave via FPRD.
//                SM1 is the slave→master mailbox. Bit 3 (0x08) = slave has
//                data waiting for the master to read (Emergency objects live
//                here after an unclean session). Uses ECT_REG_SM1STAT
//                directly - NOT ecx_mbxempty() which reads SM0STAT (wrong
//                direction: master→slave).
//
// dumpSlaveState: logs SOEM slavelist fields set by ecx_config_map_group().
//                 Call after configMap to verify PDO byte counts and DC state.
//
// dumpPreConfigMapState: captures live ESC state immediately before configMap.
//                 Fires on every attempt - comparing crash vs success cases
//                 reveals whether slaves are actually in PreOp when configMap
//                 is called. live_state from FPRD on 0x0130 (not from
//                 slavelist[i].state cache). Mismatch line fires if they differ.
//                 Port fields (lastidx, redstate) show SOEM's frame-index state.
//
// probeDiagRegisters: probes optional ESC registers 0x098E (sync error counter)
//                 and 0x09AE (SYNC0 pulse counter) on slave 1. Returns a struct
//                 indicating which are implemented. Call once before the pumps.
//                 wkc=0 means the register does not exist on this ESC; data from
//                 it would be meaningless and is excluded from pump samples.
//
// samplePumpState: reads live registers per slave during pump loops (after
//                 waitUntil so timing is not affected). Reads 0x0130 (AL Status),
//                 0x0134 (AL Status Code), 0x0981 (DC Sync Act) via FPRD - not
//                 from cached slavelist[i].state. Optional registers included
//                 only if probeDiagRegisters confirmed they are implemented.
//                 Returns false if any slave shows non-zero al_code (anomaly).
// ============================================================

// Concrete non-template SEH wrapper for ecx_err2string.
// Template safeCall cannot reliably protect this call: MSVC inlines the template
// __try into drainElistImpl (which has std::string temporaries from strf calls),
// producing broken SEH frame setup so the exception escapes instead of being caught.
// A concrete __declspec(noinline) function with only C-style locals (no destructors)
// keeps the __try in an isolated frame that always catches the exception.
#ifdef _WIN32
__declspec(noinline)
#endif
static const char* safeErr2String(ec_errort err, uint32_t* exCode) noexcept
{
    *exCode = 0;
#ifdef _WIN32
    __try
    {
        return ecx_err2string(err);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        *exCode = static_cast<uint32_t>(GetExceptionCode());
        return nullptr;
    }
#else
    return ecx_err2string(err);
#endif
}

// Shared implementation - maxEntries is injectable so unit tests can
// exercise the bailout path with a real (non-corrupted) elist by using a low cap.
//
// Per-entry sanity guard: with a corrupted master state the drain can iterate
// past the valid elist into adjacent memory, reading ec_errort entries with
// garbage Slave indices (monotonically decreasing values - the signature of
// reading sequential memory as struct). The known cause (an
// EC_MAXMBX/EC_MBXPOOLSIZE/EC_MAXEEPDO header ABI mismatch) is fixed at the
// source; the guard below is defence-in-depth so any similar condition aborts
// cleanly with a diagnostic rather than logging 100 bogus entries.
//
// Valid ranges (from ec_type.h):
//   Slave : [0, EC_MAXSLAVE]   (0 = master-level error)
//   Etype : 0..9 (defined values: 0,1,3,4,9; allow the gaps for forward compat)
static void drainElistImpl(ecx_contextt* ctx, const char* location, int maxEntries)
{
    int count = 0;
    ec_errort err;
    bool corrupted = false;
    while (ecx_iserror(ctx) && count < maxEntries)
    {
        ecx_poperror(ctx, &err);
        ++count;

        // Sanity-check the popped entry before formatting. Garbage values are the
        // signature of out-of-bounds reads on a corrupted master state - log once
        // and abort the drain so we don't compound the issue with 99 more reads.
        if (err.Slave > EC_MAXSLAVE || static_cast<int>(err.Etype) < 0 ||
            static_cast<int>(err.Etype) > 9)
        {
            Logger::instance().logDiag(strf(
                "DIAG | elist | location=%s | slave=%d | etype=%d | "
                "status=corrupted_skip | entries_before_abort=%d",
                location, err.Slave, static_cast<int>(err.Etype), count));
            LOG_WARNING(strf(
                "EtherCATMaster: drainElist aborted at entry %d - out-of-range "
                "Slave=%d / Etype=%d indicates elist corruption. Location=%s",
                count, err.Slave, static_cast<int>(err.Etype), location));
            corrupted = true;
            break;
        }

        uint32_t fmtEx = 0;
        const char* errStr = nullptr;
        // Pre-check SDO abort code before calling safeErr2String: ec_sdoerror2string(0)
        // returns null, which causes ecx_err2string to strncpy into null → AV. Catching
        // that AV 30+ times per g4 drain loop causes non-deterministic heap corruption
        // (STATUS_HEAP_CORRUPTION 0xc0000374) in Release builds. Avoid entirely.
        if (err.Etype != EC_ERR_TYPE_SDO_ERROR || ec_sdoerror2string(err.AbortCode) != nullptr)
            errStr = safeErr2String(err, &fmtEx);
        Logger::instance().logDiag(strf(
            "DIAG | elist | location=%s | slave=%d | etype=%d | %s",
            location, err.Slave, static_cast<int>(err.Etype),
            (fmtEx == 0 && errStr) ? errStr : "(fmt_error)"));
    }

    if (corrupted)
    {
        // Summary already logged inside the loop; nothing further.
    }
    else if (count >= maxEntries)
    {
        Logger::instance().logDiag(strf(
            "DIAG | elist | location=%s | total=%d | status=bailout_max_reached",
            location, count));
        LOG_WARNING(strf(
            "EtherCATMaster: drainElist bailed at %d entries - elist may be corrupted "
            "or excessive emergencies queued. Location=%s",
            maxEntries, location));
    }
    else
    {
        Logger::instance().logDiag(strf(
            "DIAG | elist | location=%s | total=%d | status=%s",
            location, count, count == 0 ? "empty" : "drained"));
    }
}

static void drainElist(ecx_contextt* ctx, const char* location)
{
    drainElistImpl(ctx, location, 100);
}

// Check each slave's ALStatusCode before critical configMap / SDO operations.
// Uses SOEM's cached slavelist[i].ALstatuscode (populated by ecx_readstate). If the cache
// shows zero, attempts a live FPRD on register 0x0134. A wkc <= 0 FPRD is treated as clean
// (no slave present or port not open in tests). Returns false on the first slave with a
// non-zero code; failedSlaveOut and detailOut are set. Returns true if all slaves are clean.
static bool checkSlaveErrorStateCached(ecx_contextt* ctx, int slaveCount, const char* location,
                                        int& failedSlaveOut, std::string& detailOut)
{
    failedSlaveOut = -1;
    detailOut.clear();
    for (int i = 1; i <= slaveCount; ++i)
    {
        uint16_t alCode = ctx->slavelist[i].ALstatuscode;
        if (alCode == 0)
        {
            // Attempt live FPRD to catch stale cache. safeCall guards against crashes
            // on an uninitialized port (e.g. in unit tests with a zeroed ecx_contextt).
            uint32_t fprdEx = 0;
            int sz = sizeof(alCode);
            int wkc = -1;
            PlatformRT::safeCall([&]()
            {
                wkc = ecx_FPRD(&ctx->port, ctx->slavelist[i].configadr,
                                ECT_REG_ALSTATCODE, sz, &alCode, EC_TIMEOUTRET);
            }, &fprdEx);
            if (fprdEx != 0 || wkc <= 0)
                alCode = 0;
        }
        if (alCode != 0)
        {
            const char* desc = ec_ALstatuscode2string(alCode);
            Logger::instance().logDiag(strf(
                "DIAG | al_error | location=%s | slave=%d | alcode=0x%04x | desc=%s",
                location, i, alCode, desc ? desc : "?"));
            failedSlaveOut = i;
            detailOut = strf("Slave %d: ALStatusCode=0x%04x (%s) at %s -- ESC in error state.",
                             i, alCode, desc ? desc : "?", location);
            return false;
        }
    }
    return true;
}

static void dumpMbxState(ecx_contextt* ctx, int slaveCount, const char* location)
{
    for (int i = 1; i <= slaveCount; ++i)
    {
        uint8_t sm1stat = 0;
        int wkc = ecx_FPRD(&ctx->port, ctx->slavelist[i].configadr,
                            ECT_REG_SM1STAT, sizeof(sm1stat), &sm1stat, EC_TIMEOUTRET);
        if (wkc <= 0)
        {
            Logger::instance().logDiag(strf(
                "DIAG | mbx_state | location=%s | slave=%d | sm1stat=FAILED | wkc=%d",
                location, i, wkc));
        }
        else
        {
            bool pending = (sm1stat & 0x08) != 0;
            Logger::instance().logDiag(strf(
                "DIAG | mbx_state | location=%s | slave=%d | sm1stat=0x%02x | bit3=%d | sm1=%s",
                location, i, sm1stat, pending ? 1 : 0,
                pending ? "pending" : "empty"));
        }
    }
}

static void dumpSlaveState(ecx_contextt* ctx, int slaveCount, const char* location)
{
    for (int i = 1; i <= slaveCount; ++i)
    {
        const ec_slavet& s = ctx->slavelist[i];
        // coembxoverrun is included: SOEM increments this when an
        // emergency / SDO response arrives but the in-buffer was already
        // marked full. Non-zero is an early-warning of mailbox-pool pressure
        // (worth raising EC_MBXPOOLSIZE if seen sustained).
        Logger::instance().logDiag(strf(
            "DIAG | slave_state | location=%s | slave=%d | state=0x%02x"
            " | mbx_l=%d | mbx_rl=%d | mbx_proto=0x%04x | Obytes=%d | Ibytes=%d"
            " | hasdc=%d | DCcycle=%d | DCshift=%d | coembxoverrun=%d",
            location, i, s.state,
            s.mbx_l, s.mbx_rl, s.mbx_proto, s.Obytes, s.Ibytes,
            s.hasdc ? 1 : 0, s.DCcycle, s.DCshift,
            s.coembxoverrun));
    }
}

static void dumpPreConfigMapState(ecx_contextt* ctx, int slaveCount)
{
    // Port-level state - one line for the whole context.
    Logger::instance().logDiag(strf(
        "DIAG | pre_config_map_port | slavecount=%d | lastidx=%d | redstate=%d",
        ctx->slavecount, (int)ctx->port.lastidx, ctx->port.redstate));

    // Per-slave: compare live ESC state (FPRD on 0x0130) against SOEM's cache.
    for (int i = 1; i <= slaveCount; ++i)
    {
        uint16_t liveState = 0;
        int wkc = ecx_FPRD(&ctx->port, ctx->slavelist[i].configadr,
                           0x0130, sizeof(liveState), &liveState, EC_TIMEOUTRET);
        uint8_t cachedState = ctx->slavelist[i].state;
        Logger::instance().logDiag(strf(
            "DIAG | pre_config_map_slave | slave=%d | configadr=0x%04x"
            " | live_state=0x%04x | live_wkc=%d | cached_state=0x%02x"
            " | mbx_l=%d | mbx_rl=%d | mbx_proto=0x%04x",
            i, ctx->slavelist[i].configadr,
            liveState, wkc, cachedState,
            ctx->slavelist[i].mbx_l, ctx->slavelist[i].mbx_rl, ctx->slavelist[i].mbx_proto));

        uint8_t liveRaw  = liveState & 0xFF;
        uint8_t cachedRaw = cachedState & ~EC_STATE_ERROR;
        if (wkc > 0 && liveRaw != cachedRaw)
            Logger::instance().logDiag(strf(
                "DIAG | pre_config_map_mismatch | slave=%d | live=0x%02x | cached=0x%02x",
                i, liveRaw, cachedRaw));
    }
}

struct ProbedRegs { bool has098E = false; bool has09AE = false; };

static ProbedRegs probeDiagRegisters(ecx_contextt* ctx, int slaveCount)
{
    ProbedRegs p;
    if (slaveCount < 1) return p;

    uint16_t v = 0;
    p.has098E = (ecx_FPRD(&ctx->port, ctx->slavelist[1].configadr,
                          0x098E, sizeof(v), &v, EC_TIMEOUTRET) > 0);
    Logger::instance().logDiag(strf(
        "DIAG | reg_probe | reg=0x098E | desc=sync_error_counter | slave=1 | result=%s",
        p.has098E ? "implemented" : "not_implemented"));

    p.has09AE = (ecx_FPRD(&ctx->port, ctx->slavelist[1].configadr,
                          0x09AE, sizeof(v), &v, EC_TIMEOUTRET) > 0);
    Logger::instance().logDiag(strf(
        "DIAG | reg_probe | reg=0x09AE | desc=sync0_counter | slave=1 | result=%s",
        p.has09AE ? "implemented" : "not_implemented"));

    return p;
}

// Returns false if any slave shows non-zero al_code (anomaly detected).
static bool samplePumpState(ecx_contextt* ctx, int slaveCount,
                            const char* phase, int cycle, int wkc,
                            const ProbedRegs& probes)
{
    bool allClean = true;
    for (int i = 1; i <= slaveCount; ++i)
    {
        uint16_t alStatus = 0, alCode = 0;
        uint8_t  dcAct    = 0;
        ecx_FPRD(&ctx->port, ctx->slavelist[i].configadr, 0x0130,
                 sizeof(alStatus), &alStatus, EC_TIMEOUTRET);
        ecx_FPRD(&ctx->port, ctx->slavelist[i].configadr, ECT_REG_ALSTATCODE,
                 sizeof(alCode), &alCode, EC_TIMEOUTRET);
        ecx_FPRD(&ctx->port, ctx->slavelist[i].configadr, ECT_REG_DCSYNCACT,
                 sizeof(dcAct), &dcAct, EC_TIMEOUTRET);

        if (alCode != 0) allClean = false;

        std::string line = strf(
            "DIAG | pump_sample | phase=%s | cycle=%d | slave=%d | wkc=%d"
            " | al_status=0x%04x | al_code=0x%04x | dc_act=0x%02x",
            phase, cycle, i, wkc, alStatus, alCode, dcAct);

        if (probes.has098E)
        {
            uint16_t syncErr = 0;
            ecx_FPRD(&ctx->port, ctx->slavelist[i].configadr,
                     0x098E, sizeof(syncErr), &syncErr, EC_TIMEOUTRET);
            line += strf(" | sync_err=0x%04x", syncErr);
        }
        if (probes.has09AE)
        {
            uint16_t sync0Cnt = 0;
            ecx_FPRD(&ctx->port, ctx->slavelist[i].configadr,
                     0x09AE, sizeof(sync0Cnt), &sync0Cnt, EC_TIMEOUTRET);
            line += strf(" | sync0_cnt=0x%04x", sync0Cnt);
        }

        Logger::instance().logDiag(line);
    }
    return allClean;
}

// Safe wrappers using PlatformRT::safeCall()
// On Windows this uses __try/__except to catch SOEM access violations.
// On Linux SOEM is more stable and safeCall() is a plain call.

static int safeConfigInit(ecx_contextt* ctx, uint32_t* exceptionCode)
{
    int result = -1;
    PlatformRT::safeCall([&]() { result = ecx_config_init(ctx); }, exceptionCode);
    return result;
}

static int safeConfigMap(ecx_contextt* ctx, void* iomap, int group, uint32_t* exceptionCode)
{
    int result = -1;
    PlatformRT::safeCall([&]() { result = ecx_config_map_group(ctx, iomap, group); }, exceptionCode);
    return result;
}

static int safeSendReceive(ecx_contextt* ctx, uint32_t* exceptionCode)
{
    int result = -1;
    PlatformRT::safeCall([&]()
    {
        ecx_send_processdata(ctx);
        result = ecx_receive_processdata(ctx, EC_TIMEOUTRET);
    }, exceptionCode);
    return result;
}

// ============================================================
// CSP mode SDO write with retry and read-back
// Hardcoded to mode 8 (Cyclic Synchronous Position)
// ============================================================
static bool setCSPModeWithRetry(ecx_contextt* ctx, int slaveIndex, int maxRetries = 3)
{
    const int8_t CSP_MODE = 8;

    for (int attempt = 1; attempt <= maxRetries; ++attempt)
    {
        int8_t mode = CSP_MODE;
        int sz = sizeof(mode);
        int wkc = ecx_SDOwrite(ctx, (uint16)slaveIndex, 0x6060, 0x00,
            FALSE, sz, &mode, 700000);
        if (wkc <= 0)
        {
            LOG_WARNING(strf("  Slave %d: CSP mode SDO write attempt %d/%d failed (wkc=%d)",
                slaveIndex, attempt, maxRetries, wkc));
            osal_usleep(50000);
            continue;
        }

        // Verify by reading back mode of operation display (0x6061)
        int8_t modeDisplay = 0;
        int szRead = sizeof(modeDisplay);
        int readWkc = ecx_SDOread(ctx, (uint16)slaveIndex, 0x6061, 0x00,
            FALSE, &szRead, &modeDisplay, 700000);
        if (readWkc > 0 && modeDisplay == CSP_MODE)
        {
            LOG_INFO(strf("  Slave %d: CSP mode set and verified (attempt %d)", slaveIndex, attempt));
            return true;
        }

        if (readWkc > 0)
        {
            LOG_WARNING(strf("  Slave %d: Mode write OK but readback=%d (expected 8), attempt %d",
                slaveIndex, static_cast<int>(modeDisplay), attempt));
        }
        osal_usleep(50000);
    }

    LOG_ERROR(strf("  Slave %d: Failed to set CSP mode after %d attempts", slaveIndex, maxRetries));
    return false;
}

// ============================================================
// initialize()
// ============================================================
InitResult EtherCATMaster::initialize(const std::string& nicName)
{
    if (m_initialized)
    {
        LOG_WARNING("EtherCATMaster: Already initialized, shutting down first.");
        shutdown();
    }

    // Restart dispatch thread if shutdown() stopped it
    if (!m_pumpDispatchThread.joinable())
    {
        m_pumpDispatchStop = false;
        m_pumpDispatchReq  = false;
        startPumpDispatchThread();
    }

    // ---- SIMULATION MODE ----
    if (m_simulationMode)
    {
        LOG_INFO("EtherCATMaster: SIMULATION MODE -- no hardware required.");
        m_slaveCount = static_cast<int>(m_driveConfigs.size());
        m_drives.clear();
        for (const DriveConfig& cfg : m_driveConfigs)
        {
            auto drive = std::make_unique<A6Drive>();
            drive->setSlaveIndex(cfg.slaveIndex);
            drive->setScaling(cfg.countsPerMm);
            // Initial software limits are wide-open (+/- 2x stroke);
            // the MotionController tightens them post-homing.
            drive->setLimits(-2.0 * cfg.strokeMm, 2.0 * cfg.strokeMm);
            drive->setName(cfg.name);
            drive->setCommandSyncCycles(m_commandSyncCycles);
            double initPos = (cfg.axisType == "belt") ? 0.0 : cfg.homingBackoffMm;
            drive->setSimPosition(initPos);
            drive->setSimTarget(initPos);
            m_drives.push_back(std::move(drive));
        }
        m_initialized = true;
        setMasterState(ECState::Op);
        LOG_INFO(strf("EtherCATMaster: Sim init complete. %d virtual drive(s).",
            static_cast<int>(m_drives.size())));
        return InitResult::success();
    }

    // ---- REAL HARDWARE ----
    // Close any NIC handle left open by a previous failed init attempt.
    // If ecx_init() was called but the init never completed (m_initialized=false),
    // the NIC handle is still live. Deleting the context without closing it first
    // leaks the handle and causes ecx_config_map_group() crashes on the next attempt.
    if (m_ecxOpen)
    {
        LOG_INFO("EtherCATMaster: Closing previous NIC handle before re-init...");
        ecx_close(ctxPtr(m_ctx));
        m_ecxOpen = false;
    }
    delete ctxPtr(m_ctx);
    ecx_contextt* ctx = new ecx_contextt();
    memset(ctx, 0, sizeof(ecx_contextt));
    m_ctx = ctx;

    LOG_INFO(strf("EtherCATMaster: Initializing on NIC: '%s'", nicName.c_str()));

    ec_adaptert* adapter = ec_find_adapters();
    m_resolvedAdapterName = nicName;  // fallback: use as-is (Linux interface name)

    for (ec_adaptert* cur = adapter; cur; cur = cur->next)
    {
        if (nicName == cur->desc || nicName == cur->name)
        {
            m_resolvedAdapterName = cur->name;  // pcap device path e.g. \\Device\\NPF_{GUID}
            LOG_INFO(strf("EtherCATMaster: Matched adapter '%s'", nicName.c_str()));
            break;
        }
    }
    ec_free_adapters(adapter);

    int ok = ecx_init(ctx, m_resolvedAdapterName.c_str());

    if (!ok)
    {
        m_lastError = strf("ecx_init() failed for '%s'. Run as Administrator, check Npcap.", nicName.c_str());
        LOG_ERROR(m_lastError);
        return InitResult::fail(InitError::NicNotFound, m_lastError);
    }

    m_ecxOpen = true;  // NIC handle is now live
    LOG_INFO("EtherCATMaster: ecx_init() OK. Discovering slaves...");

    // Drain the pcap RX buffer of any stale frames from a prior session
    // before SOEM's broadcast discovery. Stale frames cause ecx_config_init()
    // to misparse them as discovery responses, crashing with 0xc0000005.
    // The drain in discoverAndPrepareSlaves (post-PreOp) targets a different source
    // of stale frames and remains; these two drains are not redundant.
    {
        uint16_t dummy = 0;
        for (int flush = 0; flush < 8; ++flush)
        {
            ecx_BRD(&ctx->port, 0x0000, 0, sizeof(dummy), &dummy, EC_TIMEOUTRET);
        }
        LOG_INFO("EtherCATMaster: Pre-config-init Npcap drain complete.");
    }

    // Single config_init attempt (no retry -- retry corrupts SOEM state)
    uint32_t exCode = 0;
    m_slaveCount = safeConfigInit(ctx, &exCode);

    if (exCode != 0)
    {
        // ecx_init() already returned OK, so the NIC IS bound - a crash here
        // points at buffer pollution or drive ESC corruption, not binding.
        m_lastError = strf(
            "ecx_config_init() crashed (0x%08x).\n"
            "Possible causes:\n"
            "  - Stale frames in Npcap RX buffer from prior session\n"
            "  - Drive ESC in corrupted state (post-fault residual)\n"
            "Recovery: close app, wait 10 seconds, retry.\n"
            "If problem persists: power-cycle drives, then retry.",
            exCode);
        LOG_ERROR(m_lastError);
        ecx_close(ctx);
        return InitResult::fail(InitError::NicBindFailed, m_lastError, -1, exCode);
    }

    if (m_slaveCount <= 0)
    {
        m_lastError = "No EtherCAT slaves found. Check drive power and the EtherCAT cable.";
        LOG_ERROR(m_lastError);
        ecx_close(ctx);
        m_ecxOpen = false;   // handle is closed -- without this, the next init
                             // attempt double-closes it and crashes the process
        return InitResult::fail(InitError::NoSlavesFound, m_lastError);
    }

    {
        InitResult r = tryInitOnce(ctx);
        if (!r.ok)
        {
            ecx_close(ctx);
            m_ecxOpen = false;
            return r;
        }
    }

    // tryInitOnce() succeeded -- build drive objects from config
    m_drives.clear();
    for (const DriveConfig& cfg : m_driveConfigs)
    {
        if (cfg.slaveIndex < 1 || cfg.slaveIndex > m_slaveCount)
        {
            LOG_WARNING(strf("Drive config slave %d out of range, skipping.", cfg.slaveIndex));
            continue;
        }
        auto drive = std::make_unique<A6Drive>();
        drive->setSlaveIndex(cfg.slaveIndex);
        drive->setScaling(cfg.countsPerMm);
        drive->setLimits(-2.0 * cfg.strokeMm, 2.0 * cfg.strokeMm);
        drive->setName(cfg.name);
        drive->setCommandSyncCycles(m_commandSyncCycles);
        m_drives.push_back(std::move(drive));
    }

    m_initialized = true;
    setMasterState(ECState::Init);
    LOG_INFO(strf("EtherCATMaster: Init complete. %d drive(s).", static_cast<int>(m_drives.size())));
    return InitResult::success();
}

// ============================================================
// tryInitOnce()
//
// Wraps tryInitOnceBody() in safeCall so any SOEM access
// violation (ecx_SDOread mailbox ping, ecx_writestate,
// ecx_FPRD AL status, ecx_configdc, etc.) is caught, logged,
// and returned as false -- rather than killing the process.
// An uncaught crash here feeds a watchdog restart loop:
// process dies mid-init, watchdog relaunches, drives reset
// to INIT, and the next attempt crashes at the same point.
//
// Returns true on success. On failure sets m_lastError and
// returns false. Does NOT call ecx_close() -- the retry loop
// owns the context lifetime.
// ============================================================
InitResult EtherCATMaster::tryInitOnce(ecx_contextt* ctx)
{
    uint32_t exCode = 0;
    InitResult result;
    PlatformRT::safeCall([this, ctx, &result]() { result = tryInitOnceBody(ctx); }, &exCode);

    if (exCode != 0)
    {
        m_lastError = strf(
            "tryInitOnce crashed (0x%08x). "
            "SOEM call failed inside PreOp/configMap sequence -- "
            "drive ESC in bad state. Will retry.",
            exCode);
        LOG_ERROR(m_lastError);
        return InitResult::fail(InitError::SEHException, m_lastError, -1, exCode);
    }
    return result;
}

// ============================================================
// Init stages called from tryInitOnceBody()
// ============================================================

InitResult EtherCATMaster::stagePreOpSettle(ecx_contextt* ctx, int timeoutMs)
{
    // Poll ecx_readstate() rather than blind-sleeping.
    // Stability criterion: two consecutive reads showing all slaves in PreOp with no
    // error bit. A blind sleep gives no EtherCAT traffic and no state verification;
    // slaves from INIT could pass the sleep and then crash configMap or drop out later.
    int stableCount = 0;
    int elapsed = 0;
    while (elapsed < timeoutMs)
    {
        osal_usleep(100000);  // 100ms poll interval
        elapsed += 100;

        uint32_t rsEx = 0;
        PlatformRT::safeCall([ctx]() { ecx_readstate(ctx); }, &rsEx);
        if (rsEx != 0)
        {
            LOG_WARNING(strf("EtherCATMaster: ecx_readstate crashed during PreOp settle (0x%08x)", rsEx));
            stableCount = 0;
            continue;
        }

        bool allClean = true;
        for (int i = 1; i <= m_slaveCount; ++i)
        {
            uint8_t s = ctx->slavelist[i].state;
            if ((s & ~EC_STATE_ERROR) != EC_STATE_PRE_OP || (s & EC_STATE_ERROR))
            { allClean = false; break; }
        }
        stableCount = allClean ? stableCount + 1 : 0;
        if (stableCount >= 2)
        {
            LOG_INFO(strf("EtherCATMaster: All slaves stable in PreOp after %dms.", elapsed));
            break;
        }
    }

    // Hard gate: any slave not cleanly in PreOp → retry init rather than crash configMap.
    bool allPreOpFinal = true;
    int firstSettleFail = -1;
    for (int i = 1; i <= m_slaveCount; ++i)
    {
        uint8_t s = ctx->slavelist[i].state;
        if ((s & ~EC_STATE_ERROR) != EC_STATE_PRE_OP || (s & EC_STATE_ERROR))
        {
            LOG_WARNING(strf("  Slave %d: state=0x%02x after settle - not cleanly in PreOp", i, s));
            allPreOpFinal = false;
            if (firstSettleFail < 0) firstSettleFail = i;
        }
    }
    if (!allPreOpFinal)
    {
        m_lastError = "One or more slaves not stably in PreOp after settle. Retrying init.";
        LOG_ERROR(m_lastError);
        return InitResult::fail(InitError::PreOpSettleFailed, m_lastError, firstSettleFail);
    }
    return InitResult::success();
}

InitResult EtherCATMaster::stagePDOConfig(ecx_contextt* ctx)
{
    // --- Assign 1702h RPDO for torque/belt drives ---
    //
    // 1702h is a fixed manufacturer PDO (19 bytes):
    //   out+0:  6040h controlword (2B)
    //   out+2:  607Ah target position (4B)
    //   out+6:  60FFh target velocity (4B)
    //   out+10: 6071h target torque (2B)
    //   out+12: 6060h mode of operation (1B)
    //   out+13: 60B8h touch probe (2B)
    //   out+15: 607Fh max speed (4B)
    //
    // Do NOT write to 1702h contents -- fixed manufacturer object.
    // Only assign it to SM2 via 1C12:01. SOEM reads size from ESI.
    // Must happen before configMap.
    for (const DriveConfig& cfg : m_driveConfigs)
    {
        if (cfg.mode != "torque") continue;
        int tidx = cfg.slaveIndex;
        LOG_INFO(strf("EtherCATMaster: Assigning 1702h RPDO for torque drive (slave %d)...", tidx));

        uint8_t zero = 0, one = 1;

        int wkc = ecx_SDOwrite(ctx, (uint16)tidx, 0x1C12, 0x00, FALSE, sizeof(zero), &zero, 700000);
        if (wkc <= 0) LOG_WARNING(strf("  Slave %d: SM2 clear wkc=%d", tidx, wkc));

        uint16_t rpdo = 0x1702;
        wkc = ecx_SDOwrite(ctx, (uint16)tidx, 0x1C12, 0x01, FALSE, sizeof(rpdo), &rpdo, 700000);
        if (wkc <= 0)
        {
            m_lastError = strf("Slave %d: Failed to assign 1702h to SM2 (wkc=%d)", tidx, wkc);
            LOG_ERROR(m_lastError);
            return InitResult::fail(InitError::PDOConfigFailed, m_lastError, tidx);
        }

        wkc = ecx_SDOwrite(ctx, (uint16)tidx, 0x1C12, 0x00, FALSE, sizeof(one), &one, 700000);
        if (wkc <= 0) LOG_WARNING(strf("  Slave %d: SM2 count set wkc=%d", tidx, wkc));

        LOG_INFO(strf("  Slave %d: 1702h assigned to SM2. SOEM will read size from ESI.", tidx));
    }

    // --- Configure DC + register PO2SO hook ---
    // ecx_configdc() must stay before ecx_config_map_group() - canonical SOEM ordering.
    // dcsyncHook fires per slave inside ecx_config_map_group() during the PreOp→SafeOp
    // transition, at the correct point relative to CoE 1C12/1C13 SDO reads.
    // ecx_dcsync0() writes only DC FPWR registers - no CoE mailbox overlap.
    {
        // Defensive SYNC0 disarm BEFORE DC configuration (companion to the
        // shutdown() disarm): if the previous session left sync
        // units armed (crash, kill, or any exit that skipped shutdown), configdc
        // would re-base ESC system time underneath a live SYNC0 comparator and the
        // pulse chaos can latch the drive's sync supervision (AL-0x0027; slave 1 /
        // reference clock is the most exposed). Disarming first makes init idempotent
        // regardless of what state the bus was left in.
        for (int i = 1; i <= m_slaveCount; ++i)
            ecx_dcsync0(ctx, (uint16)i, FALSE, 0, 0);
        LOG_INFO(strf("EtherCATMaster: SYNC0 disarmed on %d slave(s) before DC config (stale-session guard).",
                      m_slaveCount));

        LOG_INFO("EtherCATMaster: Configuring DC + registering PO2SO hook...");
        s_dcSyncNs       = (m_controlLoopHz > 0) ? (1000000000u / m_controlLoopHz) : 1000000u;
        s_dcSyncOffsetNs = m_dcSyncOffsetNs;
        if (ecx_configdc(ctx))
        {
            for (int i = 1; i <= m_slaveCount; ++i)
            {
                if (ctx->slavelist[i].hasdc)
                {
                    ctx->slavelist[i].PO2SOconfig = &dcsyncHook;
                    LOG_INFO(strf("  Slave %d: PO2SO hook registered (period=%uus offset=%dus)",
                        i, s_dcSyncNs / 1000, s_dcSyncOffsetNs / 1000));
                }
            }
        }
        else
        {
            LOG_WARNING("EtherCATMaster: ecx_configdc() failed - DC sync not configured.");
        }
    }

    // --- ALStatusCode pre-check before configMap ---
    // Uses SOEM cache from stagePreOpSettle (ecx_readstate was called in polling loop, fresh on
    // return), with live FPRD fallback per slave.
    // Non-zero ALStatusCode means the ESC is in an error state - configMap crashes if we proceed.
    {
        int failedSlave = -1;
        std::string detail;
        if (!checkSlaveErrorStateCached(ctx, m_slaveCount, "pre_config_map", failedSlave, detail))
        {
            m_lastError = detail;
            LOG_WARNING(m_lastError);
            return InitResult::fail(InitError::SlaveErrorState, m_lastError, failedSlave);
        }
    }

    // --- Build IO map ---
    // Capture live ESC state on every attempt before configMap fires.
    dumpPreConfigMapState(ctx, m_slaveCount);

    uint32_t mapExCode = 0;
    auto mapT0 = std::chrono::steady_clock::now();
    int mapResult = safeConfigMap(ctx, m_iomap, 0, &mapExCode);
    auto mapUs = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - mapT0).count();
    Logger::instance().logDiag(strf(
        "DIAG | timing | event=config_map | duration_us=%lld | result=%d",
        (long long)mapUs, mapExCode != 0 ? -1 : mapResult));
    if (mapExCode != 0)
    {
        Logger::instance().logDiag(strf(
            "DIAG | config_map_crash | exception_code=0x%08x | duration_us=%lld",
            mapExCode, (long long)mapUs));
        m_lastError = strf("ecx_config_map_group() crashed (0x%08x). "
            "Drive ESC in bad state -- power cycle required.", mapExCode);
        LOG_ERROR(m_lastError);
        return InitResult::fail(InitError::PDOConfigFailed, m_lastError, -1, mapExCode);
    }

    LOG_INFO(strf("EtherCATMaster: config_map result=%d Obytes=%d Ibytes=%d mbx_l=%d",
        mapResult,
        ctx->slavelist[1].Obytes,
        ctx->slavelist[1].Ibytes,
        ctx->slavelist[1].mbx_l));
    LOG_INFO(strf("EtherCATMaster: Slave 1 SM2_len=%d SM3_len=%d",
        ctx->slavelist[1].SM[2].SMlength,
        ctx->slavelist[1].SM[3].SMlength));

    LOG_INFO(strf("EtherCATMaster: Found %d slave(s).", m_slaveCount));
    for (int i = 1; i <= m_slaveCount; ++i)
    {
        LOG_INFO(strf("  Slave %d: '%s' (Vendor:0x%08x Product:0x%08x)",
            i,
            ctx->slavelist[i].name,
            ctx->slavelist[i].eep_man,
            ctx->slavelist[i].eep_id));
    }

    drainElist(ctx, "post_config_map");
    dumpSlaveState(ctx, m_slaveCount, "post_config_map");

    return InitResult::success();
}

InitResult EtherCATMaster::stageDCArm(ecx_contextt* ctx)
{
    // Verify dcsyncHook applied - read back DC registers per slave.
    //
    // NO re-arm on a small margin. A sub-millisecond margin here is a
    // READ-TIME ARTIFACT: the hook arms start times +SyncDelay (100ms) during
    // configMap, and by the time this stage reads them back, config latency
    // has consumed almost all of it for the earliest-armed slaves. The old
    // "margin < 1ms -> re-arm with fresh DC time" call restarted exactly
    // those slaves' pulse trains seconds before the OP request and rewrote
    // an armed SYNC0 channel at SafeOp (the manipulation this file's own
    // OP-transition doctrine forbids) -- and the instant slave-1 0x0027
    // rejections tracked those re-armed slaves and no others. A genuinely
    // dead arm (start time in the past, channel inactive) is still rescued
    // by the SYNC0-dead check immediately before the OP request.
    //
    // Each per-slave block runs under safeCall: an uncaught crash between
    // one slave's readback and the next would otherwise bypass the
    // graceful-failure path.
    int failedSlave = -1;
    uint32_t exCode = 0;
    for (int i = 1; i <= m_slaveCount; ++i)
    {
        if (!ctx->slavelist[i].hasdc) continue;
        uint16_t cfgAddr = ctx->slavelist[i].configadr;
        PlatformRT::safeCall([&]()
        {
            uint8_t  dcAct = 0xFF;
            ecx_FPRD(&ctx->port, cfgAddr, ECT_REG_DCSYNCACT, 1, &dcAct, EC_TIMEOUTRET);
            int64_t startTime = 0;
            ecx_FPRD(&ctx->port, cfgAddr, ECT_REG_DCSTART0,  8, &startTime, EC_TIMEOUTRET);
            int64_t sysTime = 0;
            ecx_FPRD(&ctx->port, cfgAddr, ECT_REG_DCSYSTIME, 8, &sysTime,   EC_TIMEOUTRET);
            int64_t marginNs = startTime - sysTime;
            LOG_INFO(strf("  Slave %d: PO2SO hook result - 0x0981=0x%02x startTime=%lld marginNs=%lld (~%dms)",
                i, dcAct, (long long)startTime, (long long)marginNs, (int)(marginNs / 1000000LL)));
            if (dcAct != 0x03)
                LOG_WARNING(strf("  Slave %d: 0x0981=0x%02x (expected 0x03) - dcsyncHook may not have applied", i, dcAct));

        }, &exCode);

        if (exCode != 0)
        {
            failedSlave = i;
            break;
        }
    }
    if (exCode != 0)
    {
        Logger::instance().logDiag(strf(
            "DIAG | dc_arm_crash | slave=%d | exception_code=0x%08x",
            failedSlave, exCode));
        return InitResult::fail(InitError::DCConfigFailed,
            strf("Slave %d: stageDCArm crashed (0x%08x). DC register read or SYNC0 re-arm faulted - power cycle required.",
                 failedSlave, exCode),
            failedSlave, exCode);
    }
    return InitResult::success();
}

InitResult EtherCATMaster::pingMailbox(ecx_contextt* ctx)
{
    // Step 3: Read device type (0x1000) - always available in PreOp.
    // If wkc=0, the CoE mailbox isn't ready yet. Retry up to 5x with 100ms gaps.
    // This is the root cause of 1C12 wkc=0: drive in PreOp but mailbox not open.
    bool mbxAlive = false;
    int pingsNeeded = 0;
    auto pingT0 = std::chrono::steady_clock::now();
    for (int ping = 1; ping <= 5; ++ping)
    {
        pingsNeeded = ping;
        uint32_t devType = 0;
        int sz = sizeof(devType);
        int wkc = ecx_SDOread(ctx, 1, 0x1000, 0x00, FALSE, &sz, &devType, 700000);
        if (wkc > 0)
        {
            LOG_INFO(strf("EtherCATMaster: Mailbox alive (ping %d, devType=0x%08x)", ping, devType));
            mbxAlive = true;
            break;
        }
        LOG_WARNING(strf("EtherCATMaster: Mailbox ping %d/5 wkc=%d -- waiting 100ms...", ping, wkc));
        osal_usleep(100000);
    }
    auto pingUs = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - pingT0).count();
    Logger::instance().logDiag(strf(
        "DIAG | timing | event=mbx_ping | attempts=%d | duration_us=%lld | result=%s",
        pingsNeeded, (long long)pingUs, mbxAlive ? "ok" : "failed"));

    if (!mbxAlive)
    {
        m_lastError = "CoE mailbox not responding after 5 pings in PreOp. Drive may need power cycle.";
        return InitResult::fail(InitError::SlaveNotResponding, m_lastError, 1);
    }
    return InitResult::success();
}

InitResult EtherCATMaster::discoverAndPrepareSlaves(ecx_contextt* ctx, int& outTimeoutMs)
{
    // --- Step 1: Read per-slave states, clear any error bits ---
    for (int i = 1; i <= m_slaveCount; ++i)
        ecx_statecheck(ctx, i, 0, 100000);

    bool anyError = false;
    for (int i = 1; i <= m_slaveCount; ++i)
    {
        uint8_t slaveState = ctx->slavelist[i].state & ~EC_STATE_ERROR;
        if (ctx->slavelist[i].state & EC_STATE_ERROR)
        {
            LOG_WARNING(strf("  Slave %d: error bit set (state=0x%02x) -- driving to INIT to clear",
                i, ctx->slavelist[i].state));
            anyError = true;
        }
        else
        {
            LOG_INFO(strf("  Slave %d: state=0x%02x", i, slaveState));
        }
    }

    if (anyError)
    {
        // Write PreOp directly to clear error bits - do NOT broadcast INIT.
        //
        // Broadcasting INIT instead sends drives to Init → next writestate
        // triggers a fresh Init→PreOp transition → ecx_config_map_group() crashes
        // (0xc0000005) because the SII controller is still completing its reset
        // sequence from the previous crash.
        //
        // The AL error bit is cleared by writing the target state (PreOp = 0x02)
        // to the AL Control register. This does not trigger a full ESC reset and
        // leaves the SII in its current idle state - exactly what configMap needs.
        ctx->slavelist[0].state = EC_STATE_PRE_OP;
        ecx_writestate(ctx, 0);
        ecx_statecheck(ctx, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 2);
        osal_usleep(500000);  // 500ms settle after error clear
        LOG_INFO(strf("EtherCATMaster: Error state cleared. Slaves in PreOp (0x%02x)",
            ctx->slavelist[0].state));
    }

    // --- Step 2: Request PreOp, verify per-slave ---
    // Record which slaves were already in PreOp before we write the broadcast.
    // If a slave just transitioned from INIT, its internal ESC state machines
    // (SM/PDI setup) need extra settling time after the AL Status reports PreOp.
    std::vector<bool> wasAlreadyPreOp(m_slaveCount + 1, false);
    for (int i = 1; i <= m_slaveCount; ++i)
        wasAlreadyPreOp[i] = ((ctx->slavelist[i].state & ~EC_STATE_ERROR) == EC_STATE_PRE_OP);

    ctx->slavelist[0].state = EC_STATE_PRE_OP;
    ecx_writestate(ctx, 0);

    // Verify each slave individually -- broadcast can show PreOp while
    // individual slaves are still transitioning
    bool allPreOp = true;
    int firstPreOpFail = -1;
    for (int i = 1; i <= m_slaveCount; ++i)
    {
        ecx_statecheck(ctx, i, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 2);
        uint8_t s = ctx->slavelist[i].state & ~EC_STATE_ERROR;
        if (s != EC_STATE_PRE_OP)
        {
            LOG_WARNING(strf("  Slave %d: failed to reach PreOp (state=0x%02x)",
                i, ctx->slavelist[i].state));
            allPreOp = false;
            if (firstPreOpFail < 0) firstPreOpFail = i;
        }
    }

    LOG_INFO(strf("EtherCATMaster: PreOp broadcast state: 0x%04x", ctx->slavelist[0].state));

    if (!allPreOp)
    {
        m_lastError = "One or more slaves failed to reach PreOp.";
        return InitResult::fail(InitError::SlaveNotResponding, m_lastError, firstPreOpFail);
    }

    // --- Step 2b: Drain Npcap receive buffer before any SDO operations ---
    // After a crash or unclean close, the NIC's Npcap receive buffer holds stale
    // EtherCAT response frames from the previous session. The first ecx_SDOread()
    // would receive one of these stale frames, misidentify it as a response to the
    // new request, and dereference a pointer from the old context → 0xc0000005.
    // ecx_BRD() sends a broadcast read and receives all queued responses in one
    // round-trip, flushing the buffer. Repeat a few times to clear burst residue.
    // An undrained buffer is what produces a first-attempt crash ~2ms after PreOp.
    {
        uint16_t dummy = 0;
        for (int flush = 0; flush < 8; ++flush)
        {
            ecx_BRD(&ctx->port, 0x0000, 0, sizeof(dummy), &dummy, EC_TIMEOUTRET);
        }
        LOG_INFO("EtherCATMaster: Npcap frame buffer flushed.");
    }

    // SM1 state snapshot before first SDO.
    // Dirty drives (previous unclean session) may have queued CoE Emergency
    // objects in SM1. If any slave shows sm1=pending here, those objects can
    // consume mailbox-pool slots during the first SDO exchange.
    // EC_MBXPOOLSIZE in the installed headers must match the value the SOEM
    // library was built with - a mismatch is a silent ABI break.
    dumpMbxState(ctx, m_slaveCount, "pre_mbx_ping");

    // Drain queued emergencies from SM1 before any SDO exchange.
    // Placed here - after ecx_statecheck confirmed PreOp (SM1 hardware enabled)
    // and after the BRD flush (stale frames cleared). No additional settle
    // needed: SM1 is readable as soon as PreOp is confirmed; the drain is read-only
    // (SM1 status FPRD + data FPRD), not an SDO operation. The 1000ms SDO settle
    // applies to write operations and comes after the mailbox ping.
    for (int i = 1; i <= m_slaveCount; ++i)
    {
        int drained = drainSlaveMailbox(ctx, i);
        LOG_INFO(strf("EtherCATMaster: Slave %d mailbox drain: %d message(s) flushed.", i, drained));
    }
    drainElist(ctx, "pre_mbx_drain");

    { auto r = pingMailbox(ctx); if (!r.ok) return r; }

    // Calculate settle timeout based on whether any slave just transitioned from INIT.
    bool anyFreshTransition = false;
    for (int i = 1; i <= m_slaveCount; ++i)
        if (!wasAlreadyPreOp[i]) { anyFreshTransition = true; break; }
    outTimeoutMs = anyFreshTransition ? 1000 : 200;
    LOG_INFO(strf("EtherCATMaster: Polling for PreOp stability (timeout=%dms, %s).",
        outTimeoutMs, anyFreshTransition ? "fresh INIT->PreOp transition" : "drives already in PreOp"));

    return InitResult::success();
}

InitResult EtherCATMaster::tryInitOnceBody(ecx_contextt* ctx)
{
    int timeoutMs = 0;
    { auto r = discoverAndPrepareSlaves(ctx, timeoutMs); if (!r.ok) return r; }
    { auto r = stagePreOpSettle(ctx, timeoutMs);         if (!r.ok) return r; }
    { auto r = stagePDOConfig(ctx);                      if (!r.ok) return r; }
    { auto r = stageDCArm(ctx);                          if (!r.ok) return r; }
    return InitResult::success();
}

InitResult EtherCATMaster::configurePDOs()
{
    if (!m_initialized) return InitResult::fail(InitError::NicNotFound, "Not initialized");
    if (m_simulationMode) { setMasterState(ECState::PreOp); return InitResult::success(); }

    LOG_INFO("EtherCATMaster: Using default PDO mappings (RPDO=1701h, TPDO=1B01h).");
    setMasterState(ECState::PreOp);
    return InitResult::success();
}

// ============================================================
// Init stages called from enterOperationalBody()
// ============================================================

InitResult EtherCATMaster::stageSafeOpEntry(ecx_contextt* ctx)
{
    // ============================================================
    // Configure PDO (SM) watchdog per slave.
    //
    // Without a watchdog, drives hold their last commanded position
    // indefinitely if the host process is killed or Windows takes a
    // multi-second scheduling break. With the watchdog enabled, each
    // slave raises ErC1.8 (resettable Class-3 fault) after
    // pdoWatchdogMs without a valid process-data frame.
    //
    // ESC register layout (EtherCAT standard, same on all ESCs):
    //   0x0400 (uint16): Watchdog Divider
    //     Each count = 40ns at 25MHz EtherCAT clock.
    //     We write 0x09C2 (2498) → (2498+2)×40ns = 100µs per tick.
    //   0x0410 (uint16): SM/PDO Watchdog Time
    //     Number of divider ticks before timeout.
    //     0 = disabled. 1000 ticks × 100µs = 100ms.
    // ============================================================
    if (m_pdoWatchdogMs > 0)
    {
        // Divider: 0x09C2 = 2498 → (2498+2) × 40ns = 100µs per tick
        uint16_t wdDivider = 0x09C2;
        // Ticks = ms × 10 (since 1ms / 100µs per tick = 10 ticks/ms)
        uint16_t wdTime = static_cast<uint16_t>(
            std::min(m_pdoWatchdogMs * 10, 65535));

        for (int i = 1; i <= m_slaveCount; ++i)
        {
            int r1 = ecx_FPWR(&ctx->port, ctx->slavelist[i].configadr,
                              0x0400, sizeof(wdDivider), &wdDivider, EC_TIMEOUTRET);
            int r2 = ecx_FPWR(&ctx->port, ctx->slavelist[i].configadr,
                              0x0410, sizeof(wdTime),    &wdTime,    EC_TIMEOUTRET);
            if (r1 > 0 && r2 > 0)
                LOG_INFO(strf("  Slave %d: PDO watchdog set to %dms (%d ticks)", i, m_pdoWatchdogMs, wdTime));
            else
                LOG_WARNING(strf("  Slave %d: PDO watchdog write failed (r1=%d r2=%d) -- "
                    "drive will hold position indefinitely on host crash", i, r1, r2));
        }
    }
    else
    {
        // Explicitly write 0 to 0x0410 on each slave to disable the watchdog in hardware.
        // Skipping this write leaves whatever value was set by a previous run active.
        uint16_t wdDisable = 0;
        for (int i = 1; i <= m_slaveCount; ++i)
        {
            ecx_FPWR(&ctx->port, ctx->slavelist[i].configadr,
                     0x0410, sizeof(wdDisable), &wdDisable, EC_TIMEOUTRET);
        }
        LOG_INFO("EtherCATMaster: PDO watchdog disabled (0x0410=0 written to all slaves).");
    }

    // ============================================================
    // Per-drive mode selection via SDO (CSP=8, PP=1)
    // PP drives also get profile velocity/accel/decel written here.
    // These must be set before cyclic exchange starts -- no SDO mid-session.
    // ============================================================
    LOG_INFO("EtherCATMaster: Setting drive modes via SDO...");
    for (auto& drive : m_drives)
    {
        int idx = drive->getSlaveIndex();
        if (idx < 1 || idx > m_slaveCount) continue;

        // Find matching drive config
        const DriveConfig* cfg = nullptr;
        for (const DriveConfig& dc : m_driveConfigs)
            if (dc.slaveIndex == idx) { cfg = &dc; break; }

        bool usePP = cfg && (cfg->mode == "pp");

        if (usePP)
        {
            // Write PP mode (1)
            int8_t ppMode = 1;
            int sz = sizeof(ppMode);
            int wkc = ecx_SDOwrite(ctx, (uint16)idx, 0x6060, 0x00, FALSE, sz, &ppMode, 700000);
            if (wkc <= 0)
            {
                LOG_ERROR(strf("  Slave %d: Failed to set PP mode -- falling back to CSP", idx));
                if (!setCSPModeWithRetry(ctx, idx, 3))
                {
                    m_lastError = strf("Slave %d: CSP fallback failed -- aborting", idx);
                    LOG_ERROR(m_lastError);
                    return InitResult::fail(InitError::SafeOpEntryFailed, m_lastError, idx);
                }
                drive->setPPMode(false);
                continue;
            }

            // Verify
            int8_t modeDisplay = 0;
            int szRead = sizeof(modeDisplay);
            ecx_SDOread(ctx, (uint16)idx, 0x6061, 0x00, FALSE, &szRead, &modeDisplay, 700000);
            if (modeDisplay != 1)
            {
                LOG_WARNING(strf("  Slave %d: PP mode write OK but readback=%d, retrying CSP",
                    idx, static_cast<int>(modeDisplay)));
                if (!setCSPModeWithRetry(ctx, idx, 3))
                {
                    m_lastError = strf("Slave %d: CSP fallback failed -- aborting", idx);
                    LOG_ERROR(m_lastError);
                    return InitResult::fail(InitError::SafeOpEntryFailed, m_lastError, idx);
                }
                drive->setPPMode(false);
                continue;
            }
            LOG_INFO(strf("  Slave %d: PP mode set and verified", idx));

            // Profile velocity/accel/decel are written by applyPPProfile()
            applyPPProfile(ctx, idx, *cfg);

            drive->setPPMode(true);
        }
        else if (drive->isTorqueMode())
        {
            // Torque drive using 1702h RPDO -- mode is written via PDO (6060h at out+12)
            // each cycle in setTargetTorque(). Set CST (10) via SDO here as initial state
            // before cyclic exchange starts. PDO will reinforce it every cycle.
            int8_t cstMode = 10;
            int sz = sizeof(cstMode);
            int wkc = ecx_SDOwrite(ctx, (uint16)idx, 0x6060, 0x00, FALSE, sz, &cstMode, 700000);
            if (wkc <= 0)
            {
                LOG_WARNING(strf("  Slave %d: CST mode SDO init failed (wkc=%d) -- PDO will set it each cycle", idx, wkc));
            }
            else
            {
                LOG_INFO(strf("  Slave %d: CST mode (10) set via SDO, will be reinforced via PDO each cycle", idx));
            }
        }
        else
        {
            if (!setCSPModeWithRetry(ctx, idx, 3))
            {
                m_lastError = strf("Slave %d: CSP mode failed -- aborting", idx);
                LOG_ERROR(m_lastError);
                return InitResult::fail(InitError::SafeOpEntryFailed, m_lastError, idx);
            }
            drive->setPPMode(false);
        }

        // Read back the ACTUAL mode of operation (0x6061) so the log states
        // what the drive is in, not what we asked for. This is the guard against
        // the "config said cst, log looked like CST, but the drive ran CSP"
        // trap: the confirmed mode here is read from the drive itself. (The PP
        // branch above also reads back; this covers every drive uniformly.)
        // For torque drives the SDO-set CST is reinforced by the 6060h PDO each
        // cycle in OP, so a non-CST readback here just means the PDO will set it.
        {
            int8_t modeDisp = 0;
            int szRd = sizeof(modeDisp);
            int rwkc = ecx_SDOread(ctx, (uint16)idx, 0x6061, 0x00, FALSE, &szRd, &modeDisp, 700000);
            const char* mName =
                (modeDisp == 8)  ? "CSP" :
                (modeDisp == 1)  ? "PP"  :
                (modeDisp == 10) ? "CST" :
                (modeDisp == 9)  ? "CSV" :
                (modeDisp == 6)  ? "Homing" :
                (modeDisp == 3)  ? "ProfileVel" : "other";
            if (rwkc > 0)
                LOG_INFO(strf("  Slave %d '%s': operating mode = %s (0x6061=%d, confirmed from drive)",
                    idx, drive->getName().c_str(), mName, static_cast<int>(modeDisp)));
            else
                LOG_WARNING(strf("  Slave %d: could not read back operating mode (0x6061, wkc=%d)",
                    idx, rwkc));
        }
    }

    return InitResult::success();
}

void EtherCATMaster::runCapabilityScan(ecx_contextt* ctx)
{
    // ============================================================
    // Drive capability scan (optional -- enable in config.json)
    // 24+ SDO reads per drive -- purely diagnostic, not used for
    // motor control. Off by default to reduce init time and the
    // number of SDO transactions during the unstable post-INIT window.
    // ============================================================
    if (!m_enableCapabilityScan)
    {
        LOG_INFO("EtherCATMaster: Capability scan skipped (enableCapabilityScan=false).");
        return;
    }

    LOG_INFO("EtherCATMaster: Running drive capability scan...");
    for (int i = 1; i <= m_slaveCount; ++i)
    {
        LOG_INFO(strf("  === Slave %d capabilities ===", i));

        uint32_t supportedModes = 0;
        int sz = sizeof(supportedModes);
        if (ecx_SDOread(ctx, (uint16)i, 0x6502, 0x00, FALSE, &sz, &supportedModes, 700000) > 0)
        {
            LOG_INFO(strf("  Supported modes: 0x%08x", supportedModes));
            LOG_INFO(strf("    Profile Position  (1): %s", supportedModes & 0x0001 ? "YES" : "no"));
            LOG_INFO(strf("    Profile Velocity  (3): %s", supportedModes & 0x0004 ? "YES" : "no"));
            LOG_INFO(strf("    Profile Torque    (4): %s", supportedModes & 0x0008 ? "YES" : "no"));
            LOG_INFO(strf("    Homing            (6): %s", supportedModes & 0x0020 ? "YES" : "no"));
            LOG_INFO(strf("    Interpolated Pos  (7): %s", supportedModes & 0x0040 ? "YES" : "no"));
            LOG_INFO(strf("    CSP               (8): %s", supportedModes & 0x0080 ? "YES" : "no"));
            LOG_INFO(strf("    CSV               (9): %s", supportedModes & 0x0100 ? "YES" : "no"));
            LOG_INFO(strf("    CST              (10): %s", supportedModes & 0x0200 ? "YES" : "no"));
        }

        // Probe safety-related registers
        struct RegProbe { uint16_t index; uint8_t sub; const char* name; int size; };
        RegProbe probes[] = {
            { 0x607D, 0x01, "SW position limit min", 4 },
            { 0x607D, 0x02, "SW position limit max", 4 },
            { 0x6065, 0x00, "Following error window", 4 },
            { 0x6072, 0x00, "Max torque", 2 },
            { 0x6080, 0x00, "Max motor speed", 4 },
            { 0x6071, 0x00, "Target torque (CST)", 2 },
            { 0x6077, 0x00, "Torque actual value", 2 },
            { 0x6078, 0x00, "Current actual value", 2 },
            { 0x60B8, 0x00, "Touch probe function", 2 },
            { 0x6081, 0x00, "Profile velocity", 4 },
            { 0x6083, 0x00, "Profile acceleration", 4 },
            { 0x6084, 0x00, "Profile deceleration", 4 },
        };

        for (const auto& p : probes)
        {
            uint8_t buf[4] = {};
            int rsz = p.size;
            int wkc = ecx_SDOread(ctx, (uint16)i, p.index, p.sub, FALSE, &rsz, buf, 700000);
            if (wkc > 0)
            {
                uint32_t val = 0;
                memcpy(&val, buf, std::min(p.size, 4));
                LOG_INFO(strf("    0x%04x:%02x %-28s = %u (0x%0*x) SUPPORTED",
                    p.index, p.sub, p.name, val, p.size * 2, val));
            }
            else
            {
                LOG_INFO(strf("    0x%04x:%02x %-28s NOT AVAILABLE",
                    p.index, p.sub, p.name));
            }
        }
    }
    LOG_INFO("EtherCATMaster: Capability scan complete.");
}

InitResult EtherCATMaster::stagePreOpPump(ecx_contextt* ctx)
{
    // NOT idempotent: calls timerBegin() (refcounted multimedia timer - must be
    // paired with timerEnd() in stageOPTransition). Only reachable via
    // enterOperationalBody(), which has no retry loop at this stage.

    // Fresh ecx_readstate + ALStatusCode pre-check before SDO burst.
    // stageSafeOpEntry does not call ecx_readstate, so the cache may be stale here.
    // Abort early with SlaveErrorState rather than sending SDOs into an error ESC.
    {
        uint32_t rsEx = 0;
        PlatformRT::safeCall([ctx]() { ecx_readstate(ctx); }, &rsEx);
        if (rsEx != 0)
        {
            m_lastError = strf("stagePreOpPump: ecx_readstate crashed (0x%08x) -- aborting.", rsEx);
            LOG_ERROR(m_lastError);
            return InitResult::fail(InitError::SEHException, m_lastError, -1, rsEx);
        }
        int failedSlave = -1;
        std::string detail;
        if (!checkSlaveErrorStateCached(ctx, m_slaveCount, "pre_op_pump_entry", failedSlave, detail))
        {
            m_lastError = detail;
            LOG_WARNING(m_lastError);
            return InitResult::fail(InitError::SlaveErrorState, m_lastError, failedSlave);
        }
    }

    // ============================================================
    // 0x6072 max torque + CST speed limits (per-mode).
    //
    // ALL drives: 0x6072 = 3000 (300.0% = motor peak). Rated torque is the
    // CONTINUOUS/thermal number; peak exists for transient acceleration. A
    // blanket 1000 (the drive's DS402 shipping default) is not an engineered
    // cap and hurts both modes: belt tension silently clips at 100%, and
    // vertical heave transients demanding >rated saturate -> following-error
    // spikes + softened dynamics. Real protection lives elsewhere: drive i2t
    // (thermal), the following-error fault (runaway; tighten
    // followingErrorWindowMm for a crisper guard), and the command-side
    // accel/braking clamps.
    // NOTE: torque drives also need C06.20 (runaway protection) = 0 -- the
    // check misreads a driver dragging the belt against its torque as
    // runaway and latches Er06.0 (manual p181). That is a
    // ONE-TIME provisioning write (drive_profiles torqueOnly section /
    // panel), stored in drive NVRAM -- deliberately NOT re-written here.
    // ============================================================
    for (int i = 1; i <= m_slaveCount; ++i)
    {
        uint16_t maxTorque = 3000;   // 0.1% units: motor peak, all modes
        int sz = sizeof(maxTorque);
        int wkc = ecx_SDOwrite(ctx, (uint16)i, 0x6072, 0x00, FALSE, sz, &maxTorque, 700000);
        if (wkc > 0)
            LOG_INFO(strf("  Slave %d: 0x6072 max torque = %.1f%% (peak; transient headroom)", i,
                          maxTorque / 10.0));
        else
            LOG_WARNING(strf("  Slave %d: 0x6072 max torque write FAILED (wkc=%d)", i, wkc));
    }

    // Write the PER-AXIS following-error window (0x6065) to each drive.
    // SafeOp phase -- the safe SDO window, no cyclic conflict (NOT on the RT
    // thread mid-OP). Each axis carries its own window (rig config).
    // WRITE-ONLY: a readback-verify (SDOread after each write) doubles the
    // mailbox SDO traffic in the SafeOp->OP window and destabilises init on
    // hardware (intermittent "Slave N not in OP"). A gated/diagnostic readback
    // can run OUTSIDE this window if verification is wanted.
    {
        LOG_INFO("EtherCATMaster: Writing per-axis following-error window (0x6065)...");
        for (int i = 1; i <= m_slaveCount; ++i)
        {
            double windowMm = 0.0, cpm = 0.0;
            for (const DriveConfig& dc : m_driveConfigs)
                if (dc.slaveIndex == i) { windowMm = dc.followingErrorWindowMm; cpm = dc.countsPerMm; break; }

            if (windowMm <= 0.0) { LOG_INFO(strf("  Slave %d: following-error window disabled (0)", i)); continue; }
            if (cpm <= 0.0)      { LOG_WARNING(strf("  Slave %d: no countsPerMm -- skipping 0x6065", i)); continue; }

            uint32_t feCounts = static_cast<uint32_t>(windowMm * cpm);
            int sz = sizeof(feCounts);
            int wkc = ecx_SDOwrite(ctx, (uint16)i, 0x6065, 0x00, FALSE, sz, &feCounts, 700000);
            if (wkc > 0)
                LOG_INFO(strf("  Slave %d: 0x6065 = %.1fmm (%u counts)", i, windowMm, feCounts));
            else
                LOG_WARNING(strf("  Slave %d: Failed to write 0x6065 (wkc=%d)", i, wkc));
        }
    }

    // Drain SOEM elist after SafeOp SDO writes (mode, torque, following
    // error window). Catches any internally recorded failures that returned
    // wkc>0 but pushed an error into the elist.
    drainElist(ctx, "post_safeop_sdos");

    // Optional diagnostic SDO reads - extracted to runCapabilityScan() to keep this stage ≤150 lines
    runCapabilityScan(ctx);

    // --- Initialize drive targets ---
    for (auto& drive : m_drives)
    {
        drive->setTargetPosition(0.0);
    }

    // Write DS402 Shutdown controlword (0x0006) before the pre-OP pump.
    //
    // iomap is zeroed at construction, so m_pControlword points at 0x0000 after
    // assignPDOPointers(). DS402 interprets 0x0000 as DisableVoltage / Quick Stop
    // (bit 2 = Quick Stop = 0 is an active Quick Stop command). A previously-enabled
    // drive receiving 0x0000 on OP entry transitions through Quick Stop Active →
    // potential fault. A cold drive receiving 0x0000 is driven toward Switch On
    // Disabled instead of the intended Ready to Switch On state.
    //
    // 0x0006 = Enable Voltage (bit 1) + Quick Stop (bit 2) = DS402 "Shutdown"
    // command, which requests "Ready to Switch On" state. The drive is not enabled
    // (Switch On bit 0 = 0, Enable Operation bit 3 = 0), so no motion occurs.
    // This is the correct neutral state to hold throughout the pump. The control
    // loop's first enableOperation() call transitions from here to Operation Enabled.
    for (auto& drive : m_drives)
    {
        drive->prepareForPump();
    }
    LOG_INFO("EtherCATMaster: Controlword initialized to 0x0006 (DS402 Shutdown/Ready-to-Switch-On).");

    // --- DC-sync-aware pump before OP request ---
    //
    // Rather than pumping a fixed number of iterations, we pump until the drive
    // hardware reports DC is actually synchronized. We read AL status register
    // 0x0134 (per-slave via FPRD) every 100ms. When all slaves show 0x0000,
    // DC is locked. This eliminates the fixed-time guesswork that caused
    // AL 0x0027 ("SYNC starting time set incorrectly") on cold starts.
    //
    // Minimum pump: 500ms (drives need at least a few SYNC0 cycles to stabilize)
    // Maximum pump: 15s (timeout; if DC hasn't locked, something is wrong)
    // Pump at 1ms cadence.
    // Probe optional ESC registers once before pumps begin.
    ProbedRegs probes = probeDiagRegisters(ctx, m_slaveCount);

    PlatformRT::timerBegin();
    const int64_t ticksPer1Ms = PlatformRT::periodCounts(1000.0);  // 1ms - pre-OP and OP transition pumps
    PlatformRT::Timestamp deadline = PlatformRT::now();

    int lastWkc = -999;  // sentinel - triggers wkc_change log on first cycle
    LOG_INFO("EtherCATMaster: Pumping cyclic data for 4s at 1ms cadence (4000 cycles)...");
    for (int cycle = 0; cycle < 4000; ++cycle)
    {
        PlatformRT::advancePeriod(deadline, ticksPer1Ms);
        uint32_t ex = 0;
        int wkc = safeSendReceive(ctx, &ex);
        if (ex != 0)
        {
            m_lastError = strf("Process data crash during pump (0x%08x) -- power cycle drive.", ex);
            LOG_ERROR(m_lastError);
            PlatformRT::timerEnd();
            ecx_close(ctx);
            m_ecxOpen = false;
            m_initialized = false;
            return InitResult::fail(InitError::PreOpPumpFailed, m_lastError, -1, ex);
        }
        PlatformRT::waitUntil(deadline);

        // Post-waitUntil diagnostics - no cadence impact.
        if (wkc != lastWkc)
        {
            Logger::instance().logDiag(strf(
                "DIAG | wkc_change | phase=pre_op_pump | cycle=%d | prev_wkc=%d | new_wkc=%d",
                cycle + 1, lastWkc, wkc));
            lastWkc = wkc;
        }
        if ((cycle + 1) % 100 == 0)
        {
            bool clean = samplePumpState(ctx, m_slaveCount, "pre_op_pump", cycle + 1, wkc, probes);
            if (!clean)
                Logger::instance().logDiag(strf(
                    "DIAG | pump_anomaly | phase=pre_op_pump | first_anomaly_cycle=%d | resolution_ms=100",
                    cycle + 1));
        }

        // DC margin sample every 500 cycles (every 500ms at 1ms cadence).
        // Reads ECT_REG_DCSYSTIME (0x0910, 64-bit ns) from slave 1 and computes time
        // remaining until the next SYNC0 boundary using the configured DCcycle period.
        if (m_slaveCount >= 1 && (cycle + 1) % 500 == 0)
        {
            int64_t dcSysTime = 0;
            int dcWkc = ecx_FPRD(&ctx->port, ctx->slavelist[1].configadr,
                                  ECT_REG_DCSYSTIME, sizeof(dcSysTime), &dcSysTime, EC_TIMEOUTRET);
            int64_t dcCycle = ctx->slavelist[1].DCcycle;
            int64_t marginNs = (dcWkc > 0 && dcCycle > 0)
                               ? (dcCycle - (dcSysTime % dcCycle))
                               : -1;
            Logger::instance().logDiag(strf(
                "DIAG | dc_margin_sample | phase=pre_op_pump | cycle=%d | wkc=%d"
                " | dc_sys_time_ns=%lld | dc_cycle_ns=%lld | margin_ns=%lld",
                cycle + 1, dcWkc,
                (long long)dcSysTime, (long long)dcCycle, (long long)marginNs));
        }
    }

    // Full snapshot after pre-OP pump, before writing OP state.
    drainElist(ctx, "pre_op_request");
    dumpMbxState(ctx, m_slaveCount, "pre_op_request");
    dumpSlaveState(ctx, m_slaveCount, "pre_op_request");
    LOG_INFO("EtherCATMaster: Pre-OP pump complete -- requesting OP.");
    return InitResult::success();
}

void EtherCATMaster::logOPFailureDiagnostics(ecx_contextt* ctx)
{
    m_lastError = "Failed to reach OP state.";
    LOG_ERROR(m_lastError);
    for (int i = 1; i <= m_slaveCount; ++i)
    {
        uint16_t alCode = 0;
        int sz = sizeof(alCode);
        ecx_FPRD(&ctx->port, ctx->slavelist[i].configadr, ECT_REG_ALSTATCODE, sz, &alCode, EC_TIMEOUTRET);
        uint16_t alState = 0xFFFF;
        sz = sizeof(alState);
        ecx_FPRD(&ctx->port, ctx->slavelist[i].configadr, 0x0130, sz, &alState, EC_TIMEOUTRET);
        uint8_t dcAct = 0xFF;
        ecx_FPRD(&ctx->port, ctx->slavelist[i].configadr, ECT_REG_DCSYNCACT, 1, &dcAct, EC_TIMEOUTRET);
        const char* alDesc = ec_ALstatuscode2string(alCode);
        LOG_ERROR(strf("  Slave %d: ALState=0x%04x ALCode=0x%04x (%s) 0x0981=0x%02x (cached state=0x%02x)",
            i, alState, alCode, alDesc ? alDesc : "?", dcAct, ctx->slavelist[i].state & ~EC_STATE_ERROR));
    }
    checkSlaveStates();
}

void EtherCATMaster::runOPBridge(ecx_contextt* ctx, int64_t ticksPer2Ms)
{
    // Warm-up bridge - run 10 frames on the calling thread before creating
    // the pump thread. Windows/Npcap per-thread send-path state
    // (OVERLAPPED handle, completion port binding) appears to be initialized
    // lazily on first use: the calling thread has sent thousands of frames,
    // a fresh OS thread has not. Without the bridge, the pump thread can
    // crash on its first safeSendReceive (~4ms after OP) with 0xC0000005,
    // timing consistent with the send path failing before the first packet
    // reaches the wire. Diagnosis is behavioral inference - not verified at
    // the Npcap source level.
    PlatformRT::Timestamp bNext = PlatformRT::now();
    for (int i = 0; i < 10; ++i)
    {
        PlatformRT::advancePeriod(bNext, ticksPer2Ms);
        uint32_t ex = 0;
        safeSendReceive(ctx, &ex);
        if (ex != 0) break;
        PlatformRT::waitUntil(bNext);
    }
}

InitResult EtherCATMaster::stageOPTransition(ecx_contextt* ctx)
{
    // NOT idempotent: calls timerEnd() (paired with timerBegin() in stagePreOpPump).
    // Only reachable via enterOperationalBody(), which has no retry loop at this stage.
    const int64_t ticksPer1Ms = PlatformRT::periodCounts(1000.0);  // OP transition pump

    // Probe optional ESC registers for the OP pump diagnostics.
    // Re-probed independently from stagePreOpPump: two FPRD reads per slave,
    // non-destructive, returns the same result as the pre-OP probe.
    ProbedRegs probes = probeDiagRegisters(ctx, m_slaveCount);

    // --- Pre-OP SYNC0 health check (Er74.1 / AL 0x0027) ---
    //
    // Do NOT unconditionally re-arm SYNC0 here. Two facts make that wrong:
    // (1) a live SYNC0's DCSTART0 (0x0990) always reads within one cycle of
    // now -- the ESC advances it to the next pulse as pulses fire -- so a
    // sub-cycle margin at OP entry is the signature of a HEALTHY running
    // sync, not a fault risk; and (2) a re-arm rewrites a LIVE sync's start
    // time to now+100ms, silencing the pulse train for 100ms at SafeOp right
    // before the OP request -- if a drive's SafeOp->OP sync check lands in
    // that gap it refuses OP with AL 0x0027 (panel Er74.1, "no sync
    // signal"); the DC reference clock (slave 1) is the most exposed.
    // Rule: never manipulate a live SYNC0 at/above SafeOp.
    //
    // The re-arm is therefore CONDITIONAL, keeping only the defensive rescue:
    // a slave whose SYNC0 is armed with a plausible start time is left strictly
    // alone; only a genuinely dead arm (0x0981 not armed, or start time stuck
    // >= 2 cycles in the past, i.e. pulses stopped advancing) is re-armed.
    // FPRD/FPWR register access only (no mailbox/SDO), init thread, one-shot.
    // The per-slave verdict is logged so each boot self-verifies.
    for (int i = 1; i <= m_slaveCount; ++i)
    {
        if (!ctx->slavelist[i].hasdc) continue;
        uint16_t cfgAddr = ctx->slavelist[i].configadr;
        uint8_t  dcAct = 0;
        int64_t startPre = 0, sysPre = 0;
        ecx_FPRD(&ctx->port, cfgAddr, ECT_REG_DCSYNCACT, 1, &dcAct,    EC_TIMEOUTRET);
        ecx_FPRD(&ctx->port, cfgAddr, ECT_REG_DCSTART0,  8, &startPre, EC_TIMEOUTRET);
        ecx_FPRD(&ctx->port, cfgAddr, ECT_REG_DCSYSTIME, 8, &sysPre,   EC_TIMEOUTRET);
        const int64_t marginPre = startPre - sysPre;

        const bool armed = (dcAct == 0x03);
        const bool live  = armed && (marginPre > -2 * (int64_t)s_dcSyncNs);
        if (live)
        {
            LOG_INFO(strf(
                "  Slave %d: SYNC0 healthy before OP (0x0981=0x%02x margin=%lld ns) -- left untouched",
                i, dcAct, (long long)marginPre));
            continue;
        }

        ecx_dcsync0(ctx, (uint16)i, TRUE, s_dcSyncNs, s_dcSyncOffsetNs);
        int64_t startPost = 0, sysPost = 0;
        ecx_FPRD(&ctx->port, cfgAddr, ECT_REG_DCSTART0,  8, &startPost, EC_TIMEOUTRET);
        ecx_FPRD(&ctx->port, cfgAddr, ECT_REG_DCSYSTIME, 8, &sysPost,   EC_TIMEOUTRET);
        LOG_WARNING(strf(
            "  Slave %d: SYNC0 DEAD before OP (0x0981=0x%02x margin pre=%lld ns) -- re-armed, margin post=%lld ns",
            i, dcAct, (long long)marginPre, (long long)(startPost - sysPost)));
    }

    // --- Request OP, keep pumping at 1ms for 5 seconds ---
    LOG_INFO("EtherCATMaster: Requesting OP...");
    ctx->slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(ctx, 0);

    PlatformRT::Timestamp deadline = PlatformRT::now();
    bool allOp = false;
    bool pumpCrashed = false;
    int firstAnomalyCycle = -1;
    int lastWkc = -999;  // sentinel - triggers wkc_change log on first cycle

    for (int i = 0; i < 5000; ++i)
    {
        PlatformRT::advancePeriod(deadline, ticksPer1Ms);
        uint32_t ex = 0;
        int wkc = safeSendReceive(ctx, &ex);
        if (ex != 0)
        {
            LOG_ERROR(strf("EtherCATMaster: Process data crash during OP transition (0x%08x)", ex));
            pumpCrashed = true;
            break;
        }
        PlatformRT::waitUntil(deadline);

        // Post-waitUntil diagnostics - no cadence impact.
        if (wkc != lastWkc)
        {
            Logger::instance().logDiag(strf(
                "DIAG | wkc_change | phase=op_pump | cycle=%d | prev_wkc=%d | new_wkc=%d",
                i + 1, lastWkc, wkc));
            lastWkc = wkc;
        }
        // Dense first 100 cycles (every 10ms); then every 100ms thereafter.
        bool doSample = (i < 100) ? ((i + 1) % 10 == 0) : ((i + 1) % 100 == 0);
        if (doSample)
        {
            bool clean = samplePumpState(ctx, m_slaveCount, "op_pump", i + 1, wkc, probes);
            if (!clean && firstAnomalyCycle < 0)
            {
                firstAnomalyCycle = i + 1;
                int resMs = (i < 100) ? 10 : 100;
                Logger::instance().logDiag(strf(
                    "DIAG | pump_anomaly | phase=op_pump | first_anomaly_cycle=%d | resolution_ms=%d",
                    firstAnomalyCycle, resMs));
            }
        }

        // Early-exit check every 100 cycles (every 100ms at 1ms cadence).
        // Uses ecx_readstate (safeCall) to avoid stalling the pump with the 50ms
        // timeout that ecx_statecheck would impose. Breaks out as soon as all
        // slaves confirm OP, avoiding unnecessary pump cycles.
        if ((i + 1) % 100 == 0)
        {
            uint32_t rsEx = 0;
            PlatformRT::safeCall([ctx]() { ecx_readstate(ctx); }, &rsEx);
            if (rsEx != 0)
            {
                LOG_ERROR(strf("stageOPTransition: ecx_readstate crashed at cycle %d (0x%08x)",
                               i + 1, rsEx));
                pumpCrashed = true;
                break;
            }
            bool allOpNow = true;
            for (int j = 1; j <= m_slaveCount; ++j)
            {
                if ((ctx->slavelist[j].state & ~EC_STATE_ERROR) != EC_STATE_OPERATIONAL)
                {
                    allOpNow = false;
                    break;
                }
            }
            if (allOpNow)
            {
                allOp = true;
                Logger::instance().logDiag(strf(
                    "DIAG | op_early_exit | cycle=%d | all_slaves_op=1", i + 1));
                break;
            }
        }
    }
    PlatformRT::timerEnd();

    // Final state check - broadcast then per-slave to confirm
    if (!pumpCrashed)
    {
        uint32_t rsEx = 0;
        PlatformRT::safeCall([ctx]() { ecx_readstate(ctx); }, &rsEx);
        if (rsEx != 0)
        {
            LOG_ERROR(strf("stageOPTransition: ecx_readstate crashed (0x%08x)", rsEx));
        }
        else
        {
            allOp = true;
            for (int i = 1; i <= m_slaveCount; ++i)
            {
                if ((ctx->slavelist[i].state & ~EC_STATE_ERROR) != EC_STATE_OPERATIONAL)
                {
                    allOp = false;
                    break;
                }
            }
        }
    }

    // Full snapshot after OP attempt - before returning, regardless of outcome.
    drainElist(ctx, "post_op_result");
    dumpSlaveState(ctx, m_slaveCount, "post_op_result");

    Logger::instance().logDiag(strf(
        "DIAG | pump_summary | first_anomaly_cycle=%s | allOp=%d | pumpCrashed=%d",
        firstAnomalyCycle >= 0 ? std::to_string(firstAnomalyCycle).c_str() : "none",
        allOp ? 1 : 0, pumpCrashed ? 1 : 0));

    if (!allOp)
    {
        logOPFailureDiagnostics(ctx);  // sets m_lastError
        return InitResult::fail(InitError::OPTransitionFailed, m_lastError);
    }

    // Verify PDO transport is flowing before declaring success.
    // Drives may report ALState=OPERATIONAL while WKC is still low.
    PlatformRT::timerBegin();
    InitResult wkcResult = verifyOperationalWKC(ctx);
    PlatformRT::timerEnd();
    if (!wkcResult)
        return wkcResult;

    setMasterState(ECState::Op);
    LOG_INFO("EtherCATMaster: OPERATIONAL.");

    startPump();  // dispatch thread runs bridge + launches m_pumpThread

    return InitResult::success();
}

// ============================================================
// After all slaves confirm OPERATIONAL, pump for
// m_wkcValidationCycles at 1ms cadence and require at least
// m_wkcValidationThreshold fraction of cycles to show
// WKC == m_expectedWKC. This catches the failure mode
// where drives reach ALState=0x08 but PDO transport is not yet
// flowing (WKC below expected).
// ============================================================
InitResult EtherCATMaster::verifyOperationalWKC(ecx_contextt* ctx)
{
    const int64_t ticksPer1Ms = PlatformRT::periodCounts(1000.0);
    const int cycles = (m_wkcValidationCycles > 0) ? m_wkcValidationCycles : 50;
    const double threshold = m_wkcValidationThreshold;

    int goodCycles = 0;
    PlatformRT::Timestamp next = PlatformRT::now();

    for (int i = 0; i < cycles; ++i)
    {
        PlatformRT::advancePeriod(next, ticksPer1Ms);
        uint32_t ex = 0;
        int wkc = safeSendReceive(ctx, &ex);
        PlatformRT::waitUntil(next);

        if (ex != 0)
        {
            LOG_ERROR(strf("verifyOperationalWKC: process data crash at cycle %d (0x%08x)", i + 1, ex));
            return InitResult::fail(InitError::OPTransitionFailed,
                "WKC validation: process data crash");
        }
        if (wkc == m_expectedWKC)
            ++goodCycles;

        Logger::instance().logDiag(strf(
            "DIAG | wkc_validate | cycle=%d | wkc=%d | expected=%d | good=%d",
            i + 1, wkc, m_expectedWKC, goodCycles));
    }

    double fraction = (cycles > 0) ? static_cast<double>(goodCycles) / cycles : 0.0;
    Logger::instance().logDiag(strf(
        "DIAG | wkc_validate_result | good=%d | total=%d | fraction=%.3f | threshold=%.3f | pass=%d",
        goodCycles, cycles, fraction, threshold, fraction >= threshold ? 1 : 0));

    if (fraction < threshold)
    {
        std::string msg = strf(
            "WKC validation failed: %.0f%% of cycles had WKC=%d, need %.0f%% (expected WKC=%d). "
            "PDO transport not established.",
            fraction * 100.0, m_expectedWKC, threshold * 100.0, m_expectedWKC);
        LOG_ERROR("EtherCATMaster: " + msg);
        return InitResult::fail(InitError::OPTransitionFailed, msg);
    }

    LOG_INFO(strf("EtherCATMaster: WKC validation passed (%.0f%% good, %d/%d cycles, WKC=%d).",
        fraction * 100.0, goodCycles, cycles, m_expectedWKC));
    return InitResult::success();
}

// ============================================================
// enterOperational() wraps the real work in safeCall so that
// any ecx_SDOwrite/ecx_SDOread access violation is caught, logged,
// and returned as false - instead of silently killing the process
// and feeding the watchdog relaunch loop with no log entry
// indicating what went wrong.
// ============================================================
InitResult EtherCATMaster::enterOperational()
{
    if (!m_initialized) return InitResult::fail(InitError::NicNotFound, "Not initialized");
    if (m_simulationMode)
    {
        // Test injection: return a staged failure if one was set via setSimulatedStageError().
        if (m_simulatedStageError.has_value() && !m_simulatedStageError->ok)
        {
            m_lastError = m_simulatedStageError->detail;
            return *m_simulatedStageError;
        }
        setMasterState(ECState::Op);
        return InitResult::success();
    }

    uint32_t exCode = 0;
    InitResult result;
    PlatformRT::safeCall([this, &result]() { result = enterOperationalBody(); }, &exCode);

    if (exCode != 0)
    {
        m_lastError = strf(
            "enterOperational() crashed (0x%08x). "
            "An SDO write to the drive failed - drive ESC in bad state. "
            "Power cycle the drive and retry.",
            exCode);
        LOG_ERROR(m_lastError);
        // Force-clear state so the next initialize() creates a fresh context
        // rather than attempting ecx_close() on a potentially corrupt handle.
        m_initialized = false;
        m_ecxOpen = false;
        setMasterState(ECState::Error);
        return InitResult::fail(InitError::SEHException, m_lastError, -1, exCode);
    }
    return result;
}

InitResult EtherCATMaster::enterOperationalBody()
{
    ecx_contextt* ctx = ctxPtr(m_ctx);

    // --- Request SAFE-OP ---
    LOG_INFO("EtherCATMaster: Requesting SAFE-OP...");
    ctx->slavelist[0].state = EC_STATE_SAFE_OP;
    ecx_writestate(ctx, 0);
    {
        auto safeopT0 = std::chrono::steady_clock::now();
        ecx_statecheck(ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
        auto safeopUs = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - safeopT0).count();
        Logger::instance().logDiag(strf(
            "DIAG | timing | event=safeop_statecheck | duration_us=%lld | state=0x%02x",
            (long long)safeopUs, ctx->slavelist[0].state));
    }
    if (ctx->slavelist[0].state != EC_STATE_SAFE_OP)
    {
        m_lastError = strf("Failed SAFE-OP. State=0x%02x", ctx->slavelist[0].state);
        LOG_ERROR(m_lastError);
        setMasterState(ECState::SafeOp);
        return InitResult::fail(InitError::SafeOpEntryFailed, m_lastError);
    }
    setMasterState(ECState::SafeOp);
    LOG_INFO("EtherCATMaster: SAFE-OP reached.");

    // dcsync0 applied via PO2SO hook inside ecx_config_map_group() - no call here.
    m_expectedWKC = (ctx->grouplist[0].outputsWKC * 2) + ctx->grouplist[0].inputsWKC;
    LOG_INFO(strf("EtherCATMaster: Expected WKC=%d", m_expectedWKC));
    assignPDOPointers();
    Logger::instance().logDiag(strf(
        "DIAG | pdo_pointers_assigned | slaveCount=%d | expectedWKC=%d | masterState=%d",
        m_slaveCount, m_expectedWKC, m_masterState.load(std::memory_order_relaxed)));

    { auto r = stageSafeOpEntry(ctx);  if (!r.ok) return r; }
    { auto r = stagePreOpPump(ctx);    if (!r.ok) return r; }
    { auto r = stageOPTransition(ctx); if (!r.ok) return r; }

    // Initialise SOEM cyclic mailbox handler AFTER all synchronous
    // init SDO writes are done (mode-of-operation, max torque, following error
    // window). Registering a slave via ecx_slavembxcyclic re-routes mailbox
    // responses through the cyclic-handler queue, so subsequent synchronous
    // ecx_SDOwrite/read calls in the same slave can't collect their replies
    // and return wkc=0. Ordering: all synchronous SDO traffic in
    // stagePreOpPump completes, drives reach OP via stageOPTransition,
    // THEN the cyclic handler takes over the mailbox routing.
    initCyclicMailboxHandler(ctx);

    return InitResult::success();
}

void EtherCATMaster::stopPump()
{
    // Wait for any in-flight dispatch to finish. m_dispatchBusy is set by
    // startPump() inside the lock (before notify) and cleared by T4 after
    // startPumpBody() returns. Without this wait, stopPump() can return with
    // m_pumpActive=false while T4 is mid-bridge, about to set it true and
    // launch T5 concurrent with T6.
    while (m_dispatchBusy.load(std::memory_order_acquire))
        std::this_thread::yield();

    if (m_pumpActive.load())
    {
        m_pumpActive.store(false);
        if (m_pumpThread.joinable())
            m_pumpThread.join();
        LOG_INFO("EtherCATMaster: Background pump stopped.");
    }
}

// Persistent dispatch thread - always running (started in constructor,
// stopped in shutdown/destructor). startPump() posts a request here; this thread
// calls startPumpBody() so the bridge and pump thread never run on a dying RT thread.
void EtherCATMaster::startPumpDispatchThread()
{
    m_pumpDispatchThread = std::thread([this]() {
        std::unique_lock<std::mutex> lk(m_pumpDispatchMu);
        for (;;)
        {
            m_pumpDispatchCv.wait(lk, [this]() {
                return m_pumpDispatchReq || m_pumpDispatchStop;
            });
            if (m_pumpDispatchStop)
            {
                m_dispatchBusy.store(false, std::memory_order_release);  // cleared on stop-exit
                break;
            }
            m_pumpDispatchReq = false;
            lk.unlock();
            startPumpBody();
            m_dispatchBusy.store(false, std::memory_order_release);  // dispatch complete
            lk.lock();
        }
    });
}

void EtherCATMaster::startPumpBody()
{
    if (m_rtLoopActive.load(std::memory_order_acquire)) return;  // RT loop owns SOEM
    if (m_simulationMode || !m_initialized) return;
    if (m_pumpActive.load()) return;

    ecx_contextt* ctx = static_cast<ecx_contextt*>(m_ctx);
    if (!ctx) return;

    Logger::instance().logDiag(strf(
        "DIAG | start_pump_body | masterState=%d | pumpActive=%d | rtLoopActive=%d",
        m_masterState.load(std::memory_order_relaxed),
        m_pumpActive.load() ? 1 : 0,
        m_rtLoopActive.load(std::memory_order_acquire) ? 1 : 0));

    const int64_t ticksPer2Ms = PlatformRT::periodCounts(2000.0);
    // SYNC0 period (phase mod base). At the documented 500 Hz this
    // equals the pump's 2 ms cadence, so the compensator locks the pump frames to
    // the SYNC0 grid; at other rates the pump is already off-cadence vs SYNC0
    // (pre-existing) and the lock is gated OFF by default anyway.
    const int64_t pumpCycNs = (m_controlLoopHz > 0) ? (1000000000LL / m_controlLoopHz) : 2000000LL;

    // 10-frame bridge on the dispatch thread before launching the pump thread.
    // The dispatch thread has sent frames before (at init or prior restart), so
    // Npcap's per-thread send-path state is already warm - no AV on first pump frame.
    PlatformRT::timerBegin();
    runOPBridge(ctx, ticksPer2Ms);

    m_pumpActive.store(true);
    m_pumpCrashed.store(false);
    m_pumpThread = std::thread([this, ctx, ticksPer2Ms, pumpCycNs]() {
#ifdef _WIN32
        SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
#endif
        PlatformRT::timerBegin();

        // DC phase-lock for the pump cadence (default OFF -> when
        // disabled the period below is the unchanged 2 ms nominal, byte-identical
        // to the free-running pump). The pump holds OP for
        // long stretches (init/teardown bridge, idle), exactly where the ppm walk
        // accumulates onto the SYNC0 boundary. The pump bypasses sendReceive(), so
        // we sample DCtime here.
        DcPhaseLock dcLock;
        {
            DcPhaseLock::Params p;
            p.cycleNs       = pumpCycNs;
            p.nominalCounts = ticksPer2Ms;
            p.dtSec         = 0.002;          // fixed 2 ms pump cadence
            p.kp            = m_dcPhaseLockKp;
            p.ki            = m_dcPhaseLockKi;
            p.maxTrimNs     = m_dcPhaseLockMaxTrimNs;
            p.warmupCycles  = 500;
            dcLock.configure(p);
            dcLock.setEnabled(m_dcPhaseLockEnabled);
        }
        int64_t pumpPhase = DcPhaseLock::kNoSample;

        PlatformRT::Timestamp pNext = PlatformRT::now();
        while (m_pumpActive.load())
        {
            int64_t periodCounts = ticksPer2Ms;
            if (dcLock.enabled()) periodCounts = dcLock.update(pumpPhase);
            PlatformRT::advancePeriod(pNext, periodCounts);
            uint32_t ex = 0;
            safeSendReceive(ctx, &ex);
            if (ex != 0)
            {
                LOG_ERROR(strf("EtherCATMaster: Pump thread crash (0x%08x) -- EtherCAT transport down.", ex));
                m_pumpCrashed.store(true);
                break;
            }
            // Sample the DC phase for next cycle's trim (gated; off => no work).
            if (dcLock.enabled() && pumpCycNs > 0)
            {
                int64_t ph = static_cast<int64_t>(ctx->DCtime) % pumpCycNs;
                if (ph < 0) ph += pumpCycNs;
                pumpPhase = ph;
            }
            PlatformRT::waitUntil(pNext);
        }
        PlatformRT::timerEnd();
    });

    PlatformRT::timerEnd();
    LOG_INFO("EtherCATMaster: Background pump started.");
}

void EtherCATMaster::startPump()
{
    if (!m_initialized) return;
    if (m_pumpActive.load()) return;
    {
        std::lock_guard<std::mutex> lk(m_pumpDispatchMu);
        // Check stop flag under lock before setting m_dispatchBusy to avoid
        // a shutdown deadlock where T4 exits without clearing m_dispatchBusy.
        if (m_pumpDispatchStop) return;
        m_pumpDispatchReq = true;
        m_dispatchBusy.store(true, std::memory_order_release);  // set before notify
    }
    m_pumpDispatchCv.notify_one();
}

// ============================================================
// Write PP profile SDOs (0x6081/6083/6084) for a drive.
// Called from enterOperational() at init and from reapplyPPProfile()
// after ecx_recover_slave() restores OP state (which resets SDOs).
// ============================================================
void EtherCATMaster::applyPPProfile(ecx_contextt* ctx, int slaveIdx, const DriveConfig& cfg)
{
    if (cfg.countsPerMm <= 0) return;

    if (cfg.maxVelocityMmS > 0)
    {
        uint32_t profileVel = static_cast<uint32_t>(cfg.maxVelocityMmS * cfg.countsPerMm);
        int sz = sizeof(profileVel);
        int wkc = ecx_SDOwrite(ctx, (uint16)slaveIdx, 0x6081, 0x00, FALSE, sz, &profileVel, 700000);
        LOG_INFO(strf("  Slave %d: Profile velocity = %u counts/s (%.1fmm/s) wkc=%d",
            slaveIdx, profileVel, cfg.maxVelocityMmS, wkc));
    }

    if (cfg.maxAccelerationMmS2 > 0)
    {
        uint32_t profileAccel = static_cast<uint32_t>(cfg.maxAccelerationMmS2 * cfg.countsPerMm);
        int sz = sizeof(profileAccel);
        int wkc = ecx_SDOwrite(ctx, (uint16)slaveIdx, 0x6083, 0x00, FALSE, sz, &profileAccel, 700000);
        LOG_INFO(strf("  Slave %d: Profile acceleration = %u counts/s2 (%.1fmm/s2) wkc=%d",
            slaveIdx, profileAccel, cfg.maxAccelerationMmS2, wkc));

        wkc = ecx_SDOwrite(ctx, (uint16)slaveIdx, 0x6084, 0x00, FALSE, sz, &profileAccel, 700000);
        LOG_INFO(strf("  Slave %d: Profile deceleration = %u counts/s2 wkc=%d", slaveIdx, profileAccel, wkc));
    }
}

void EtherCATMaster::reapplyPPProfile(int slaveIdx)
{
    if (!m_initialized || m_simulationMode) return;

    const DriveConfig* cfg = nullptr;
    for (const DriveConfig& dc : m_driveConfigs)
        if (dc.slaveIndex == slaveIdx) { cfg = &dc; break; }

    if (!cfg || cfg->mode != "pp") return;

    LOG_INFO(strf("EtherCATMaster: Re-applying PP profile SDOs for slave %d after recovery.", slaveIdx));
    applyPPProfile(ctxPtr(m_ctx), slaveIdx, *cfg);
}

// ============================================================
// Cyclic mailbox handler integration.
//
// SOEM exposes ecx_mbxhandler() to drain pending mailbox operations (incoming
// emergency frames, outgoing queued SDOs) from a non-blocking handler called
// every cycle. Without this handler the master never sees real-time emergency
// messages from drives, only their
// indirect effects (WKC drops, eventual ErC1.1).
//
// initCyclicMailboxHandler() runs once at init time (after SAFE-OP, before the
// pre-OP pump). It initialises SOEM's mailbox queue for group 0 and registers
// each CoE-capable slave for cyclic handling.
//
// processCyclicMailbox(limit) runs every cycle from ControlLoopWorker::run(),
// right after sendReceive(). limit caps the number of mailbox ops processed
// per call to bound worst-case RT-loop latency. Default 1 (conservative);
// raise only once RT-loop latency data confirms no impact.
// ============================================================
void EtherCATMaster::initCyclicMailboxHandler(ecx_contextt* ctx)
{
    if (m_mbxHandlerInit || !ctx) return;

    uint32_t exCode = 0;
    int initRc = 0;
    PlatformRT::safeCall([&]() { initRc = ecx_initmbxqueue(ctx, 0); }, &exCode);
    if (exCode != 0)
    {
        LOG_WARNING(strf("EtherCATMaster: ecx_initmbxqueue crashed (0x%08x) - cyclic mailbox handler disabled.", exCode));
        return;
    }

    int registered = 0;
    for (int i = 1; i <= m_slaveCount; ++i)
    {
        // mbx_proto bit 2 = CoE per ec_type.h.
        if ((ctx->slavelist[i].mbx_proto & 0x04) == 0) continue;

        int rc = 0;
        PlatformRT::safeCall([&]() { rc = ecx_slavembxcyclic(ctx, (uint16)i); }, &exCode);
        if (exCode != 0)
        {
            LOG_WARNING(strf("EtherCATMaster: ecx_slavembxcyclic crashed for slave %d (0x%08x).", i, exCode));
            continue;
        }
        if (rc > 0)
        {
            LOG_INFO(strf("EtherCATMaster: Slave %d added to cyclic mailbox handler.", i));
            ++registered;
        }
    }
    Logger::instance().logDiag(strf(
        "DIAG | mbx_handler_init | initmbxqueue_rc=%d | slaves_registered=%d",
        initRc, registered));
    m_mbxHandlerInit = true;
}

int EtherCATMaster::processCyclicMailbox(int limit)
{
    if (!m_mbxHandlerInit || m_simulationMode) return 0;

    // Idle fast-path: with no SDO transfer in
    // flight, service the handler at 1/16 duty instead of every cycle. This
    // keeps m_soemAccessMutex + the ecx_mbxhandler call off the frame path
    // for ~94% of cycles; unsolicited incoming mail (EMCY) is still scanned
    // within 16 cycles (8ms at 2kHz), and fault codes ride the 603F PDO
    // anyway. While the SdoWorker holds a transfer open (begin/endMailboxWork)
    // the handler runs every cycle so transfers complete at full speed.
    if (m_mbxWorkPending.load(std::memory_order_acquire) == 0)
    {
        if ((m_mbxIdleTick++ & 0x0F) != 0) return 0;
    }

    ecx_contextt* ctx = static_cast<ecx_contextt*>(m_ctx);
    if (!ctx) return 0;

    uint32_t exCode = 0;
    int processed = 0;
    {
        std::lock_guard<std::mutex> lk(m_soemAccessMutex);
        PlatformRT::safeCall([&]() { processed = ecx_mbxhandler(ctx, 0, limit); }, &exCode);
    }
    if (exCode != 0)
    {
        // Rate-limit the crash log to avoid spamming on a persistent fault path.
        static std::atomic<uint64_t> lastCrashLog{0};
        uint64_t now = static_cast<uint64_t>(std::time(nullptr));
        if (now - lastCrashLog.load() > 1)
        {
            lastCrashLog.store(now);
            LOG_ERROR(strf("EtherCATMaster: processCyclicMailbox crashed (0x%08x). Cyclic mailbox handling disabled.", exCode));
        }
        m_mbxHandlerInit = false;
        return 0;
    }
    return processed;
}

// ============================================================
// Canonical recovery thread.
//
// The RT control loop signals via signalRecoveryNeeded() when WkcMonitor
// detects a sustained WKC error. The recovery thread polls m_needsRecovery
// every 10 ms; on rising edge, it runs doRecoveryScan() which walks the
// canonical 4-branch decision tree from SOEM's `eoe_test.c`, `ec_sample.c`,
// and `simple_ng.c` example programs:
//
//   Branch 1: state == SAFE_OP + ERROR  → write ACK
//   Branch 2: state == SAFE_OP          → request OP again
//   Branch 3: state >  NONE             → disarm SYNC0, ecx_reconfig_slave
//                                          (full walk); only if the walk
//                                          reaches SAFE_OP: re-arm DC (with
//                                          margin verify) + request OP.
//                                          Capped at kMaxRecoveryReconfigAttempts
//                                          per slave; cap resets on address
//                                          recovery (slave physically returned).
//   Branch 4: !islost && state == NONE  → mark as lost, zero PDO inputs
//   Tail:     islost && state <= INIT   → ecx_recover_slave (address recovery)
//
// Branch 3 rules: SYNC0 must be disarmed BEFORE the reconfig
// walk - ecx_reconfig_slave takes the slave INIT→PreOp→SafeOp, and doing
// that with SYNC0 still armed is the stale-arm condition that latches
// drive-side sync faults (arm/disarm only below SafeOp). Accepting
// rc >= PRE_OP instead would re-arm SYNC0 on a PreOp slave
// and request OP from PreOp (invalid AL transition), hammering a wedged
// drive every ~2s indefinitely - hence the strict SAFE_OP acceptance.
//
// All SOEM calls are wrapped in PlatformRT::safeCall + m_soemAccessMutex.
// The mutex serialises against sendReceive() and processCyclicMailbox() on
// the RT thread; SOEM's canonical examples skip this gating but Windows/Npcap
// per-thread I/O concerns (SOEM logs are not thread-safe per stream) make a brief mutex
// defensible.
// ============================================================
void EtherCATMaster::signalRecoveryNeeded()
{
    m_needsRecovery.store(true, std::memory_order_release);
}

void EtherCATMaster::startRecoveryThread()
{
    if (m_recoveryThreadRunning.load() || m_simulationMode) return;
    if (m_recoveryThread.joinable())
    {
        // Defensive: a prior run wasn't cleaned up. Join then restart.
        m_recoveryStop.store(true);
        m_recoveryThread.join();
        m_recoveryStop.store(false);
    }
    m_needsRecovery.store(false);
    m_recoveryStop.store(false);
    m_recoveryReconfigFails.assign(m_slaveCount + 1, 0);
    m_recoveryLostPending.assign(m_slaveCount + 1, 0);
    m_recoveryThreadRunning.store(true);
    m_recoveryThread = std::thread(&EtherCATMaster::recoveryThreadMain, this);
    LOG_INFO("EtherCATMaster: Recovery thread started.");
}

void EtherCATMaster::stopRecoveryThread()
{
    if (!m_recoveryThreadRunning.load()) return;
    m_recoveryStop.store(true);
    if (m_recoveryThread.joinable())
        m_recoveryThread.join();
    m_recoveryThreadRunning.store(false);
    LOG_INFO("EtherCATMaster: Recovery thread stopped.");
}

void EtherCATMaster::recoveryThreadMain()
{
    // Explicit priority + affinity setup.
    //
    // Priority: ABOVE_NORMAL ensures the thread makes timely progress under
    // Windows background load (browsers, audio, capture, etc.) without
    // competing with the TIME_CRITICAL pump/RT threads. THREAD_PRIORITY_NORMAL
    // (the std::thread default) can be preempted for tens-to-hundreds of ms,
    // which is fatal mid-ecx_reconfig_slave - the slave can fault its mailbox
    // watchdog if we don't make progress through the INIT→PreOp→SafeOp walk.
    //
    // Affinity: explicitly reset to "all cores except the RT core" so we
    // never accidentally inherit a pinned affinity from the spawning thread.
    // If a future change starts this thread from a context that has affinity
    // pinned (e.g. from the RT thread), the inheritance would put recovery
    // and RT on the same core - fighting for CPU time. Resetting explicitly
    // makes our behaviour independent of the spawner.
    //
    // MMCSS is intentionally NOT used - that's reserved for actual real-time
    // work (the pump and control loop). Windows doesn't inherit MMCSS
    // assignment across threads, so we just don't register.
#ifdef _WIN32
    HANDLE hThis = GetCurrentThread();
    if (!SetThreadPriority(hThis, THREAD_PRIORITY_ABOVE_NORMAL))
        LOG_WARNING("RecoveryThread: SetThreadPriority(ABOVE_NORMAL) failed.");

    // Affinity: any core except core 3 (the RT pin from PlatformRT::threadSetup).
    // Use the process affinity mask intersected with ~(RT core) so we never
    // request a core that's not assigned to the process.
    DWORD_PTR procMask = 0, sysMask = 0;
    if (GetProcessAffinityMask(GetCurrentProcess(), &procMask, &sysMask) && procMask)
    {
        DWORD_PTR rtCoreBit = (1ULL << 3);  // matches PlatformRT::threadSetup(coreIndex=3)
        DWORD_PTR mask = procMask & ~rtCoreBit;
        if (mask == 0) mask = procMask;  // single-core system; nothing to exclude
        if (!SetThreadAffinityMask(hThis, mask))
            LOG_WARNING("RecoveryThread: SetThreadAffinityMask failed -- running on default cores.");
    }
#endif

    while (!m_recoveryStop.load(std::memory_order_acquire))
    {
        if (m_needsRecovery.exchange(false, std::memory_order_acq_rel))
        {
            doRecoveryScan();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// ============================================================
// DS402 / CANopen error code descriptions.
// Codes from CiA 402 part 6 (drives) plus common manufacturer-specific
// codes seen on ANCTL AS715N. Unknown codes log raw hex with "unknown".
// ============================================================
const char* EtherCATMaster::ds402ErrorCodeString(uint16_t code)
{
    switch (code)
    {
        case 0x0000: return "no error";
        case 0x1000: return "generic error";
        case 0x2310: return "continuous overcurrent";
        case 0x2320: return "short circuit / earth leakage";
        case 0x3110: return "mains overvoltage";
        case 0x3120: return "mains undervoltage";
        case 0x3210: return "DC link overvoltage";
        case 0x3220: return "DC link undervoltage";
        case 0x4210: return "drive / motor overtemperature";
        case 0x4310: return "following error (legacy code)";
        case 0x5530: return "EEPROM error";
        case 0x6010: return "watchdog reset";
        case 0x6320: return "parameter error";
        case 0x7110: return "brake chopper";
        case 0x7305: return "incremental sensor 1 fault";
        case 0x7320: return "position sensor fault";
        case 0x7500: return "communication fault";
        case 0x8110: return "CAN overrun (objects lost)";
        case 0x8120: return "CAN passive mode";
        case 0x8130: return "life guard error / heartbeat error";
        case 0x8210: return "PDO not processed (length error)";
        case 0x8220: return "PDO length exceeded";
        case 0x8611: return "following error window exceeded";
        case 0x8612: return "reference limit / position limit exceeded";
        case 0x8700: return "sync error";
        case 0xFF00: return "manufacturer-specific";
        default:     return "unknown";
    }
}

void EtherCATMaster::readDriveFaultHistory(uint16_t slaveIdx)
{
    if (m_simulationMode || !m_initialized) return;
    ecx_contextt* ctx = ctxPtr(m_ctx);
    if (!ctx) return;
    if (slaveIdx < 1 || (int)slaveIdx > m_slaveCount) return;

    // State gating: SDO reads from the recovery thread can
    // race with cyclic mailbox routing; the AS715N returns wkc=0
    // on 0x603F when slaves are in SafeOp+Error. Only attempt
    // the readback when the slave is in PRE_OP, SAFE_OP, or OP without the
    // error bit set. SafeOp+Error (0x14) is skipped - the drives don't
    // respond to SDO in that state, and Branch 1 will ACK it back to plain
    // SafeOp where the read would work; let the next recovery scan iteration
    // pick it up.
    uint16_t state = ctx->slavelist[slaveIdx].state;
    bool mailboxFunctional =
        (state == EC_STATE_PRE_OP) ||
        (state == EC_STATE_SAFE_OP) ||
        (state == EC_STATE_OPERATIONAL);
    if (!mailboxFunctional)
    {
        LOG_INFO(strf("RecoveryThread: slave %d state=0x%02x - fault-history readback skipped (mailbox not in a known-good state)",
            slaveIdx, state));
        return;
    }

    // Hold the whole-transfer mutex for the entire fault-history SDO
    // sequence so the SdoWorker cannot interleave a temp/one-off transfer between
    // these reads (mailbox-interleave protection). Lock order is transfer(outer) ->
    // soemAccess(inner, taken per read below). RT-loop impact is unchanged - each
    // read still releases soemAccessMutex, so sendReceive() interleaves as before.
    std::lock_guard<std::mutex> xfer(m_sdoTransferMutex);

    // 0x6041 - DS402 statusword (always available in PreOp+; 2 bytes).
    // Read this first as a sanity check before the optional CANopen objects.
    // If 0x6041 returns wkc=0 the slave isn't responding to SDO at all, so
    // skip the rest rather than wasting timeouts.
    uint16_t statusword = 0;
    int sz = sizeof(statusword);
    uint32_t exCode = 0;
    int wkc = 0;
    {
        std::lock_guard<std::mutex> lk(m_soemAccessMutex);
        PlatformRT::safeCall([&]() {
            wkc = ecx_SDOread(ctx, slaveIdx, 0x6041, 0x00, FALSE, &sz, &statusword, EC_TIMEOUTRXM);
        }, &exCode);
    }
    if (exCode != 0)
    {
        LOG_WARNING(strf("RecoveryThread: slave %d 0x6041 SDO read crashed (0x%08x) - aborting readback", slaveIdx, exCode));
        return;
    }
    if (wkc <= 0)
    {
        LOG_INFO(strf("RecoveryThread: slave %d 0x6041 SDO read wkc=%d - slave not responding to SDO, aborting readback", slaveIdx, wkc));
        return;
    }
    LOG_INFO(strf("RecoveryThread: slave %d 0x6041 DS402 statusword = 0x%04x", slaveIdx, statusword));

    // 0x603F - current error code (DS402, 2 bytes)
    uint16_t curErr = 0;
    sz = sizeof(curErr);
    {
        std::lock_guard<std::mutex> lk(m_soemAccessMutex);
        PlatformRT::safeCall([&]() {
            wkc = ecx_SDOread(ctx, slaveIdx, 0x603F, 0x00, FALSE, &sz, &curErr, EC_TIMEOUTRXM);
        }, &exCode);
    }
    if (exCode != 0)
    {
        LOG_WARNING(strf("RecoveryThread: slave %d 0x603F SDO read crashed (0x%08x)", slaveIdx, exCode));
        return;
    }
    if (wkc > 0)
    {
        LOG_INFO(strf("RecoveryThread: slave %d 0x603F current error = 0x%04x (%s)",
            slaveIdx, curErr, ds402ErrorCodeString(curErr)));
    }
    else
    {
        LOG_INFO(strf("RecoveryThread: slave %d 0x603F SDO read wkc=%d (drive may not support)", slaveIdx, wkc));
    }

    // 0x1001 - error register (DS301, 1 byte bit-coded category)
    //   bit 0: generic  | bit 1: current     | bit 2: voltage
    //   bit 3: temperature | bit 4: communication | bit 5: device-specific
    //   bit 7: manufacturer-specific
    uint8_t errReg = 0;
    sz = sizeof(errReg);
    {
        std::lock_guard<std::mutex> lk(m_soemAccessMutex);
        PlatformRT::safeCall([&]() {
            wkc = ecx_SDOread(ctx, slaveIdx, 0x1001, 0x00, FALSE, &sz, &errReg, EC_TIMEOUTRXM);
        }, &exCode);
    }
    if (exCode == 0 && wkc > 0)
    {
        LOG_INFO(strf("RecoveryThread: slave %d 0x1001 error register = 0x%02x", slaveIdx, errReg));
    }

    // 0x1003:0 - number of stored errors (DS301)
    uint8_t numStored = 0;
    sz = sizeof(numStored);
    {
        std::lock_guard<std::mutex> lk(m_soemAccessMutex);
        PlatformRT::safeCall([&]() {
            wkc = ecx_SDOread(ctx, slaveIdx, 0x1003, 0x00, FALSE, &sz, &numStored, EC_TIMEOUTRXM);
        }, &exCode);
    }
    if (exCode != 0 || wkc <= 0) return;
    if (numStored == 0)
    {
        LOG_INFO(strf("RecoveryThread: slave %d 0x1003 history empty (no stored errors)", slaveIdx));
        return;
    }

    // 0x1003:N - each stored error: low 16 bits = code, high 16 = manuf info.
    // Cap at 5 entries to bound scan time (each SDO ~1-2ms).
    int maxRead = (numStored > 5) ? 5 : numStored;
    LOG_INFO(strf("RecoveryThread: slave %d 0x1003 history has %d entries (reading first %d)",
        slaveIdx, numStored, maxRead));
    for (int j = 1; j <= maxRead; ++j)
    {
        uint32_t entry = 0;
        sz = sizeof(entry);
        {
            std::lock_guard<std::mutex> lk(m_soemAccessMutex);
            PlatformRT::safeCall([&]() {
                wkc = ecx_SDOread(ctx, slaveIdx, 0x1003, (uint8)j, FALSE, &sz, &entry, EC_TIMEOUTRXM);
            }, &exCode);
        }
        if (exCode != 0)
        {
            LOG_WARNING(strf("RecoveryThread: slave %d 0x1003:%d SDO read crashed (0x%08x)",
                slaveIdx, j, exCode));
            return;
        }
        if (wkc <= 0) break;
        uint16_t code = static_cast<uint16_t>(entry & 0xFFFF);
        uint16_t manuf = static_cast<uint16_t>((entry >> 16) & 0xFFFF);
        LOG_INFO(strf("RecoveryThread: slave %d 0x1003:%d = 0x%08x [code=0x%04x %s | manuf=0x%04x]",
            slaveIdx, j, entry, code, ds402ErrorCodeString(code), manuf));
    }
}

int EtherCATMaster::probeSlaveALStates(uint16_t* out, int maxSlaves)
{
    // Deinit-seat bus guard: one mutex-guarded broadcast state read, AL state
    // per slave copied out. Read-only; safe from the RT thread while it owns
    // SOEM (the seat pass runs after stopPump/setRtLoopActive).
    if (!m_initialized || m_simulationMode || !out) return 0;
    ecx_contextt* ctx = ctxPtr(m_ctx);
    if (!ctx) return 0;

    uint32_t exCode = 0;
    {
        std::lock_guard<std::mutex> lk(m_soemAccessMutex);
        PlatformRT::safeCall([ctx]() { ecx_readstate(ctx); }, &exCode);
    }
    if (exCode != 0) return 0;

    int n = std::min(m_slaveCount, maxSlaves);
    for (int i = 1; i <= n; ++i)
        out[i - 1] = ctx->slavelist[i].state;
    return n;
}

void EtherCATMaster::noteReconfigFailure(int slave, int rc)
{
    if (slave >= (int)m_recoveryReconfigFails.size()) return;
    int fails = ++m_recoveryReconfigFails[slave];
    if (fails >= kMaxRecoveryReconfigAttempts)
        LOG_ERROR(strf("RecoveryThread: slave %d reconfig failed %d times (rc=%d) - "
                       "giving up until re-init or slave power-cycle; SYNC0 left disarmed",
                       slave, fails, rc));
    else
        LOG_WARNING(strf("RecoveryThread: slave %d reconfig rc=%d (< SAFE_OP), attempt %d/%d",
                         slave, rc, fails, kMaxRecoveryReconfigAttempts));
}

void EtherCATMaster::doRecoveryScan()
{
    if (!m_initialized || m_simulationMode) return;
    ecx_contextt* ctx = ctxPtr(m_ctx);
    if (!ctx) return;

    // Snapshot states under the mutex. Recovery decisions are made off
    // a consistent read; subsequent per-slave actions re-acquire briefly.
    uint32_t exCode = 0;
    {
        std::lock_guard<std::mutex> lk(m_soemAccessMutex);
        PlatformRT::safeCall([ctx]() { ecx_readstate(ctx); }, &exCode);
    }
    if (exCode != 0)
    {
        LOG_ERROR(strf("RecoveryThread: ecx_readstate crashed (0x%08x). Aborting scan.", exCode));
        return;
    }

    int slaveCount = m_slaveCount;
    uint32_t dcSyncNs = (m_controlLoopHz > 0)
        ? (1000000000u / static_cast<uint32_t>(m_controlLoopHz)) : 1000000u;

    for (int i = 1; i <= slaveCount; ++i)
    {
        if (m_recoveryStop.load(std::memory_order_acquire)) return;

        ec_slavet* s = &ctx->slavelist[i];

        // Any read above NONE clears a pending unconfirmed-lost flag: the
        // previous scan's NONE was a burst artifact, not a departure.
        if (s->state > EC_STATE_NONE && i < (int)m_recoveryLostPending.size())
            m_recoveryLostPending[i] = 0;

        if (s->state == EC_STATE_OPERATIONAL) continue;

        // Read structured fault history before deciding what to do.
        // Gates on state internally; safe to call even on dropped slaves
        // (will just skip and return). Provides "why did it fault" data
        // that pure ALstatuscode + statusword don't give us.
        readDriveFaultHistory(static_cast<uint16_t>(i));

        // Branch 1: SafeOp + Error -> ACK
        if (s->state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
        {
            std::lock_guard<std::mutex> lk(m_soemAccessMutex);
            uint16_t prev = s->state;
            s->state = EC_STATE_SAFE_OP + EC_STATE_ACK;
            PlatformRT::safeCall([ctx, i]() { ecx_writestate(ctx, (uint16)i); }, &exCode);
            LOG_INFO(strf("RecoveryThread: slave %d ACK'd from state=0x%02x (ex=0x%08x)", i, prev, exCode));
        }
        // Branch 2: SafeOp -> request OP
        else if (s->state == EC_STATE_SAFE_OP)
        {
            std::lock_guard<std::mutex> lk(m_soemAccessMutex);
            s->state = EC_STATE_OPERATIONAL;
            if (s->mbxhandlerstate == ECT_MBXH_LOST)
                s->mbxhandlerstate = ECT_MBXH_CYCLIC;
            PlatformRT::safeCall([ctx, i]() { ecx_writestate(ctx, (uint16)i); }, &exCode);
            LOG_INFO(strf("RecoveryThread: slave %d re-requested OP from SafeOp (ex=0x%08x)", i, exCode));
        }
        // Branch 3: Some other valid state -> full reconfig
        else if (s->state > EC_STATE_NONE)
        {
            // Per-slave attempt cap. A wedged drive (e.g. PreOp+Error
            // refusing the walk) must not be hammered with reconfig + OP
            // requests every scan. Give up until re-init or until the slave
            // physically drops and returns (address-recovery tail resets it).
            if (i < (int)m_recoveryReconfigFails.size()
                && m_recoveryReconfigFails[i] >= kMaxRecoveryReconfigAttempts)
                continue;

            // Disarm SYNC0 BEFORE the reconfig walk (brief FPWR writes).
            if (s->hasdc)
            {
                std::lock_guard<std::mutex> lk(m_soemAccessMutex);
                PlatformRT::safeCall([&]()
                {
                    ecx_dcsync0(ctx, (uint16)i, FALSE, 0, 0);
                }, &exCode);
            }

            int rc = -1;
            // IMPORTANT: do NOT hold m_soemAccessMutex during ecx_reconfig_slave.
            // The call has a ~600ms timeout. Holding our mutex that long would
            // block RT sendReceive() and fire the 100ms PDO watchdog on the
            // healthy slaves - turning a one-slave fault into a three-slave
            // fault. SOEM's internal port mutex already serialises wire access
            // with the RT thread's sendReceive, so we're still safe at the bus
            // layer. The downside: master-side ec_slavet fields (state, islost,
            // mbxhandlerstate) may be partially-updated when read by the RT
            // thread. Those fields are POD scalars on x86 → reads are atomic;
            // worst case is stale state for one cycle.
            PlatformRT::safeCall([&]()
            {
                rc = ecx_reconfig_slave(ctx, (uint16)i, EC_TIMEOUTRET3 * 100);  // ~600ms
            }, &exCode);
            if (exCode != 0)
            {
                LOG_ERROR(strf("RecoveryThread: ecx_reconfig_slave slave=%d crashed (0x%08x)", i, exCode));
                noteReconfigFailure(i, rc);
                continue;
            }
            if (rc >= EC_STATE_SAFE_OP)
            {
                if (i < (int)m_recoveryReconfigFails.size())
                    m_recoveryReconfigFails[i] = 0;
                std::lock_guard<std::mutex> lk(m_soemAccessMutex);
                s->islost = FALSE;
                if (s->hasdc)
                {
                    // Re-arm at SafeOp, then verify the start-time margin the
                    // same way stageDCArm does at init - a thin
                    // margin here rolls the same Er74.1 dice the init path
                    // was hardened against.
                    int64_t marginNs = 0;
                    PlatformRT::safeCall([&]()
                    {
                        ecx_dcsync0(ctx, (uint16)i, TRUE, dcSyncNs, m_dcSyncOffsetNs);
                        uint16_t cfgAddr = s->configadr;
                        int64_t startTime = 0, sysTime = 0;
                        ecx_FPRD(&ctx->port, cfgAddr, ECT_REG_DCSTART0,  8, &startTime, EC_TIMEOUTRET);
                        ecx_FPRD(&ctx->port, cfgAddr, ECT_REG_DCSYSTIME, 8, &sysTime,   EC_TIMEOUTRET);
                        marginNs = startTime - sysTime;
                        if (marginNs < 1000000LL)  // < 1ms: re-arm with fresh DC time
                        {
                            ecx_dcsync0(ctx, (uint16)i, TRUE, dcSyncNs, m_dcSyncOffsetNs);
                            ecx_FPRD(&ctx->port, cfgAddr, ECT_REG_DCSTART0,  8, &startTime, EC_TIMEOUTRET);
                            ecx_FPRD(&ctx->port, cfgAddr, ECT_REG_DCSYSTIME, 8, &sysTime,   EC_TIMEOUTRET);
                            marginNs = startTime - sysTime;
                        }
                    }, &exCode);
                    LOG_INFO(strf("RecoveryThread: slave %d reconfigured + DC re-armed (rc=%d marginNs=%lld ex=0x%08x)",
                                  i, rc, (long long)marginNs, exCode));
                }
                else
                {
                    LOG_INFO(strf("RecoveryThread: slave %d reconfigured (no DC) rc=%d", i, rc));
                }
                // PP profile reapply (no-op for CSP/torque drives). Called outside
                // the mutex section because reapplyPPProfile re-enters SDO writes
                // via applyPPProfile which itself goes through the SOEM path.
                // For now leave the recovery scan to do the reconfig and let the
                // RT loop handle PP profile via its own existing path on next
                // cycle. Note: a future iteration may move reapplyPPProfile
                // inside the lock with care for nested-lock avoidance.
                s->state = EC_STATE_OPERATIONAL;
                PlatformRT::safeCall([ctx, i]() { ecx_writestate(ctx, (uint16)i); }, &exCode);
            }
            else
            {
                noteReconfigFailure(i, rc);
            }
        }
        // Branch 4: state == NONE
        else if (!s->islost)
        {
            // ecx_statecheck has a 2ms timeout - exactly one SYNC0 period at
            // 500Hz. Don't hold m_soemAccessMutex here either; SOEM port mutex
            // handles wire serialisation. The memset of s->inputs IS under the
            // mutex below, since that touches our PDO buffer which the RT
            // thread reads (very brief, microseconds).
            PlatformRT::safeCall([&]()
            {
                ecx_statecheck(ctx, (uint16)i, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            }, &exCode);
            if (s->state == EC_STATE_NONE)
            {
                // Double-confirm before mark-lost (see header note): first
                // sighting only arms a pending flag; a second consecutive
                // scan still reading NONE executes the mark-lost path.
                if (i < (int)m_recoveryLostPending.size() && !m_recoveryLostPending[i])
                {
                    m_recoveryLostPending[i] = 1;
                    LOG_WARNING(strf("RecoveryThread: slave %d state=NONE observed -- "
                                     "awaiting confirmation next scan (ex=0x%08x)", i, exCode));
                }
                else
                {
                    if (i < (int)m_recoveryLostPending.size())
                        m_recoveryLostPending[i] = 0;
                    std::lock_guard<std::mutex> lk(m_soemAccessMutex);
                    s->islost = TRUE;
                    s->mbxhandlerstate = ECT_MBXH_LOST;
                    if (s->Ibytes && s->inputs)
                        std::memset(s->inputs, 0, s->Ibytes);
                    LOG_WARNING(strf("RecoveryThread: slave %d marked lost (confirmed on "
                                     "consecutive scans, ex=0x%08x)", i, exCode));
                }
            }
        }

        // Address recovery tail for slaves that have lost their configured address
        if (s->islost && s->state <= EC_STATE_INIT)
        {
            int rc = 0;
            // Same rationale as Branch 3: ecx_recover_slave may take up to
            // EC_TIMEOUTRET3 = 6ms (less critical than reconfig but still
            // worth releasing the mutex). SOEM port-level locking suffices.
            PlatformRT::safeCall([&]()
            {
                rc = ecx_recover_slave(ctx, (uint16)i, EC_TIMEOUTRET3);
            }, &exCode);
            if (exCode == 0 && rc)
            {
                s->islost = FALSE;
                // Slave physically dropped and returned (e.g. drive power-cycle)
                // - fresh hardware state earns fresh reconfig attempts.
                if (i < (int)m_recoveryReconfigFails.size())
                    m_recoveryReconfigFails[i] = 0;
                LOG_INFO(strf("RecoveryThread: slave %d address recovered (rc=%d)", i, rc));
            }
            else if (exCode != 0)
            {
                LOG_ERROR(strf("RecoveryThread: ecx_recover_slave slave=%d crashed (0x%08x)", i, exCode));
            }
        }
    }
    Logger::instance().logDiag(strf(
        "DIAG | recovery_scan_complete | slavecount=%d", slaveCount));
}

void EtherCATMaster::stopSdoWorker()
{
    if (m_sdoWorker) m_sdoWorker->stop();
}

void EtherCATMaster::shutdown()
{
    if (!m_initialized)
    {
        LOG_WARNING("EtherCATMaster: Not initialized, nothing to shut down.");
        return;
    }

    // Stop the SDO worker FIRST - join it so no worker transfer is
    // in flight (or can start) before any thread tears down / ecx_close frees m_ctx.
    // Mirrors the recovery-thread-first ordering below; prevents use-after-free.
    if (m_sdoWorker) m_sdoWorker->stop();

    // Shutdown ordering:
    // (1) Stop recovery thread first - no more SOEM access from a non-RT thread.
    // (2) Stop current pump thread.
    // (3) Stop dispatch thread + join - no new pump can launch after this.
    // (4) Stop pump again - catches any pump thread dispatch launched just before join.
    // (5) ecx_close().
    stopRecoveryThread();
    stopPump();

    {
        std::lock_guard<std::mutex> lk(m_pumpDispatchMu);
        m_pumpDispatchStop = true;
        m_pumpDispatchReq  = false;
    }
    m_pumpDispatchCv.notify_one();
    if (m_pumpDispatchThread.joinable())
        m_pumpDispatchThread.join();

    stopPump();  // catch any pump thread launched by dispatch just before join

    // Reset init-time flags that must NOT survive shutdown. Without this,
    // a re-init (Init → Shutdown → Init again) sees stale state and skips
    // critical setup steps.
    //
    // Specifically: initCyclicMailboxHandler short-circuits on the second
    // init if m_mbxHandlerInit is still true from the
    // first session - the new ecx_contextt's mailbox queue never gets
    // initialised and the new slavelist never gets registered. processCyclicMailbox
    // would then call ecx_mbxhandler on stale/uninitialised state.
    m_mbxHandlerInit = false;

    if (m_simulationMode)
    {
        for (auto& d : m_drives)
            d->clearPDOPointers();
        m_drives.clear();
        m_initialized = false;
        setMasterState(ECState::None);
        LOG_INFO("EtherCATMaster: Sim shutdown complete.");
        return;
    }

    ecx_contextt* ctx = ctxPtr(m_ctx);
    LOG_INFO("EtherCATMaster: Shutting down...");

    // Disable drives before clearing PDO pointers -- disableOperation()
    // writes the controlword via the PDO pointer. Clearing first leaves a
    // null pointer that disableOperation() dereferences → crash on re-init.
    disableAllDrives();

    // Null PDO pointers before ecx_close() frees the iomap.
    // stopPump() has already joined the pump thread so no RT thread
    // is running; safe to clear without a lock.
    for (auto& d : m_drives)
        d->clearPDOPointers();

    uint32_t ex = 0;
    safeSendReceive(ctx, &ex);

    ctx->slavelist[0].state = EC_STATE_INIT;
    ecx_writestate(ctx, 0);
    ecx_statecheck(ctx, 0, EC_STATE_INIT, EC_TIMEOUTSTATE);

    // Disarm SYNC0 on every slave AFTER the OP->INIT walk. Two constraints:
    // The wedge: skipping the disarm at shutdown leaves slaves sitting in
    // INIT with sync units firing off the old session's DC time; the next init
    // re-bases ESC system time under the live comparator and slave 1 (DC reference
    // clock, most violent re-base) latches sync-invalid -> AL 0x0027 / Er74.1,
    // un-clearable until a drive power cycle. Hence: disarm at shutdown (here) +
    // defensively before configdc.
    // The ORDER: disarming BEFORE leaving OP is also wrong
    // -- a DC drive in OP whose SYNC0 vanishes sees "sync lost during operation"
    // and latches Er74.2 on every de-init. Sync supervision only runs in SafeOp/OP,
    // so the disarm must come after the INIT walk; DC FPWR register writes work in
    // any AL state.
    for (int i = 1; i <= m_slaveCount; ++i)
        ecx_dcsync0(ctx, (uint16)i, FALSE, 0, 0);
    LOG_INFO(strf("EtherCATMaster: SYNC0 disarmed on %d slave(s) after INIT walk (sync supervision off).",
                  m_slaveCount));

    ecx_close(ctx);
    m_ecxOpen = false;  // NIC handle closed

    m_initialized = false;
    setMasterState(ECState::None);
    LOG_INFO("EtherCATMaster: Shutdown complete.");
}

int EtherCATMaster::sendReceive()
{
    if (!m_initialized) return -1;
    if (m_simulationMode) return m_expectedWKC;

    ecx_contextt* ctx = ctxPtr(m_ctx);
    uint32_t ex = 0;
    int wkc = -1;
    {
        // Serialise against the recovery thread's SOEM calls.
        std::lock_guard<std::mutex> lk(m_soemAccessMutex);
        wkc = safeSendReceive(ctx, &ex);
    }

    if (ex != 0)
    {
        LOG_ERROR(strf("EtherCATMaster: sendReceive crash (0x%08x)", ex));
        return -1;
    }

    // DC phase: where the reference clock sits within the cycle, captured each
    // frame. With a DC-locked loop this is ~constant; on a free-running loop it
    // walks across the whole period as the Pi clock drifts vs the DC reference.
    // Passive (modulo + atomic store); the per-second logging is diag-gated.
    if (m_controlLoopHz > 0)
    {
        int64_t cycNs = 1000000000LL / m_controlLoopHz;
        int64_t ph = static_cast<int64_t>(ctx->DCtime) % cycNs;
        if (ph < 0) ph += cycNs;
        m_dcPhaseNs.store(ph, std::memory_order_relaxed);
    }

    // One-shot raw PDO byte dump
    static bool dumpDone = false;
    if (!dumpDone)
    {
        dumpDone = true;
        if (ctx->slavelist[1].outputs && ctx->slavelist[1].inputs)
        {
            uint8_t* rx = ctx->slavelist[1].outputs;
            uint8_t* tx = ctx->slavelist[1].inputs;
            int rxLen = std::min((int)ctx->slavelist[1].Obytes, 32);
            int txLen = std::min((int)ctx->slavelist[1].Ibytes, 64);
            std::string rxHex, txHex;
            char hexbuf[8];
            for (int b = 0; b < rxLen; ++b) { snprintf(hexbuf, sizeof(hexbuf), "%02x ", rx[b]); rxHex += hexbuf; }
            for (int b = 0; b < txLen; ++b) { snprintf(hexbuf, sizeof(hexbuf), "%02x ", tx[b]); txHex += hexbuf; }
            LOG_INFO(strf("PDO RAW RX(%d): %s", rxLen, rxHex.c_str()));
            LOG_INFO(strf("PDO RAW TX(%d): %s", txLen, txHex.c_str()));
            uint16_t faultCode = *reinterpret_cast<uint16_t*>(tx + 0);
            uint16_t sw = *reinterpret_cast<uint16_t*>(tx + 2);
            int32_t  pos = *reinterpret_cast<int32_t*>(tx + 4);
            LOG_INFO(strf("PDO interpreted: faultCode=0x%04x statusword=0x%04x actualPos=%d counts",
                faultCode, sw, pos));
        }
    }

    if (wkc != m_expectedWKC)
    {
        static int errCount = 0;
        if (++errCount % 1000 == 1)
            LOG_WARNING(strf("WKC mismatch: expected %d got %d (count:%d)", m_expectedWKC, wkc, errCount));
        return -1;
    }
    return wkc;
}

A6Drive* EtherCATMaster::getDrive(int index)
{
    if (index < 0 || index >= (int)m_drives.size()) return nullptr;
    return m_drives[index].get();
}

void EtherCATMaster::enableAllDrives() { for (auto& d : m_drives) d->enableOperation(); }
void EtherCATMaster::disableAllDrives() { for (auto& d : m_drives) d->disableOperation(); }
void EtherCATMaster::resetAllFaults() { for (auto& d : m_drives) d->startFaultReset(); }

void EtherCATMaster::assignPDOPointers()
{
    if (m_simulationMode) return;
    ecx_contextt* ctx = ctxPtr(m_ctx);
    LOG_INFO("EtherCATMaster: Assigning PDO pointers...");

    for (auto& drive : m_drives)
    {
        int idx = drive->getSlaveIndex();
        if (idx < 1 || idx > m_slaveCount) continue;
        uint8_t* out = ctx->slavelist[idx].outputs;
        uint8_t* in = ctx->slavelist[idx].inputs;

        if (!out || !in) continue;

        // Check if this drive was configured for torque mode AND the PDO remap succeeded.
        // Remap success is indicated by Obytes == 4 (controlword + target_torque).
        // If remap was disabled or failed, Obytes stays at 12 (default 1701h) and we
        // fall back to position layout.
        const DriveConfig* dcfg = nullptr;
        for (const DriveConfig& dc : m_driveConfigs)
            if (dc.slaveIndex == idx) { dcfg = &dc; break; }

        bool wantsTorque = dcfg && dcfg->mode == "torque";
        bool has1702     = (ctx->slavelist[idx].Obytes == 19);

        if (wantsTorque && has1702)
        {
            // 1702h RPDO (19 bytes):
            //   out+0:  6040h controlword (2B)
            //   out+2:  607Ah target position (4B)  -- unused in torque mode
            //   out+6:  60FFh target velocity (4B)  -- unused in torque mode
            //   out+10: 6071h target torque (2B)
            //   out+12: 6060h mode of operation (1B) -- written each cycle
            // 1B01h TPDO (28 bytes): fault(2)|status(2)|pos(4)|torque(2)|...
            drive->setPDOPointers(
                reinterpret_cast<uint16_t*>(out + 0),    // controlword     (6040h)
                reinterpret_cast<int8_t*>(out + 12),      // mode select     (6060h) -- PDO writable
                nullptr,                                   // target position -- not used in CST
                reinterpret_cast<uint16_t*>(in + 2),      // statusword      (6041h)
                nullptr,                                   // mode display
                reinterpret_cast<int32_t*>(in + 4),       // actual position (6064h)
                reinterpret_cast<int16_t*>(in + 8)        // torque feedback (6077h)
            );
            drive->setTorquePDOPointer(
                reinterpret_cast<int16_t*>(out + 10),    // target torque (6071h)
                reinterpret_cast<uint32_t*>(out + 15));  // max profile velocity (607Fh) -- MUST be
                                                         // written each cycle; a zero-filled IOmap
                                                         // = "max velocity 0" = shaft pinned static
            drive->setTorqueMode(true);
            drive->setFaultCodePDOPointer(reinterpret_cast<uint16_t*>(in + 0));  // 603F fault code

            // 0x607F velocity clamp from beltMaxRpm (counts/s = rpm/60 * counts/rev).
            // Caps the slack-take-up lunge below the drive's Er46.0 overspeed fault
            // threshold; 0 = drive-native unlimited (not recommended, see Config.h).
            if (dcfg->beltMaxRpm > 0.0)
            {
                uint32_t cps = static_cast<uint32_t>(dcfg->beltMaxRpm / 60.0 * dcfg->encoderCountsPerRev);
                drive->setMaxProfileVelocityCounts(cps);
                LOG_INFO(strf("  Slave %d: CST velocity clamp (0x607F) = %.0f rpm (%u counts/s)",
                              idx, dcfg->beltMaxRpm, cps));
            }
            LOG_INFO(strf("  Slave %d '%s': Torque PDO layout (1702h, 19 bytes out). Mode writable via PDO.",
                idx, drive->getName().c_str()));
        }
        else
        {
            if (wantsTorque)
                LOG_WARNING(strf("  Slave %d: Torque mode requested but Obytes=%d (expected 19) -- "
                    "1702h assignment may have failed. Falling back to position layout.",
                    idx, ctx->slavelist[idx].Obytes));

            // 1701h RPDO (12 bytes): controlword(2) | target_position(4) | touch_probe(2) | forced_DO(4)
            // 1B01h TPDO (28 bytes): fault_code(2) | statusword(2) | actual_pos(4) | torque(2) | ...
            drive->setPDOPointers(
                reinterpret_cast<uint16_t*>(out + 0),    // controlword       (6040h)
                nullptr,                                   // mode of operation (not in 1701h)
                reinterpret_cast<int32_t*>(out + 2),      // target position   (607Ah)
                reinterpret_cast<uint16_t*>(in + 2),      // statusword        (6041h)
                nullptr,
                reinterpret_cast<int32_t*>(in + 4),       // actual position   (6064h)
                reinterpret_cast<int16_t*>(in + 8)        // torque feedback   (6077h)
            );
            drive->setFaultCodePDOPointer(reinterpret_cast<uint16_t*>(in + 0));  // 603F fault code
            LOG_INFO(strf("  Slave %d '%s': Position PDO layout (1701h/1B01h, 12 bytes out).",
                idx, drive->getName().c_str()));
        }
    }
}

bool EtherCATMaster::configureDC() { return true; }

void EtherCATMaster::checkSlaveStates()
{
    if (!m_initialized || m_simulationMode) return;
    ecx_contextt* ctx = ctxPtr(m_ctx);
    for (int i = 1; i <= m_slaveCount; ++i)
    {
        if (ctx->slavelist[i].state != EC_STATE_OPERATIONAL)
        {
            LOG_ERROR(strf("Slave %d '%s' not in OP, state=0x%02x",
                i, ctx->slavelist[i].name, ctx->slavelist[i].state));
            if (m_onSlaveError)
                m_onSlaveError(i, strf("Slave %d not in OP", i));
        }
    }
}

void EtherCATMaster::setMasterState(ECState state)
{
    // Atomic read/write - m_masterState is read by T1/T2 (getMasterState, isOperational)
    // and written here by T8 (init thread). Plain ECState assignment is formally UB.
    ECState prev = static_cast<ECState>(m_masterState.load(std::memory_order_relaxed));
    if (prev != state)
    {
        m_masterState.store(static_cast<int>(state), std::memory_order_release);
        Logger::instance().logDiag(strf(
            "DIAG | master_state_change | prev=%d | new=%d",
            static_cast<int>(prev), static_cast<int>(state)));
        if (m_onMasterStateChanged)
            m_onMasterStateChanged(state);
    }
}

// Test-accessible wrappers. drainElistForTest uses the production cap of 100;
// drainElistForTestCapped lets unit tests supply a smaller cap to exercise the
// bailout path with a real (non-corrupted) 64-entry ring.
void EtherCATMaster::drainElistForTest(ecx_contextt* ctx, const char* location)
{
    drainElistImpl(ctx, location, 100);
}

void EtherCATMaster::drainElistForTestCapped(ecx_contextt* ctx, const char* location, int max)
{
    drainElistImpl(ctx, location, max);
}

// Test accessors
bool EtherCATMaster::checkSlaveErrorStateForTest(ecx_contextt* ctx, int slaveCount,
    const char* location, int& failedSlaveOut, std::string& detailOut)
{
    return checkSlaveErrorStateCached(ctx, slaveCount, location, failedSlaveOut, detailOut);
}

void EtherCATMaster::dumpSlaveStateForTest(ecx_contextt* ctx, int slaveCount, const char* location)
{
    dumpSlaveState(ctx, slaveCount, location);
}
