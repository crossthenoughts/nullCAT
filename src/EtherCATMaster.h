// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "A6Drive.h"
#include "Config.h"
#include "Logging.h"
#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <atomic>
#include <mutex>
#include <cstdint>
#include <thread>
#include <condition_variable>
#include <optional>

static constexpr int SOEM_IOMAP_SIZE = 4096;

enum class ECState
{
    None = 0,
    Init = 1,
    PreOp = 2,
    SafeOp = 4,
    Op = 8,
    Error = 0x10
};

// Structured error surface for the init sequence.
// Each value covers a specific failure category; new values added as natural
// mappings emerge from stages (not shoehorned into existing categories).
enum class InitError
{
    None = 0,
    AlreadyInProgress,  // concurrent init guard fired
    NicNotFound,        // ecx_init() failed - NIC not found or Npcap not installed
    NicBindFailed,      // ecx_config_init() SEH crash - NIC not bound to Npcap
    NoSlavesFound,      // ecx_config_init() returned 0 slaves
    SlaveNotResponding, // slave failed PreOp broadcast or CoE mailbox not responding
    PreOpSettleFailed,  // slaves did not stabilize in PreOp within timeout
    PDOConfigFailed,    // torque RPDO SDO, AL error check, or ecx_config_map_group() failed
    DCConfigFailed,     // PO2SO hook did not apply DC sync; failedSlave populated
    SafeOpEntryFailed,  // SafeOp not reached, or drive mode SDO failed; failedSlave populated
    SlaveErrorState,    // slave reported ALStatusCode != 0 at pre-config-map or pre-OP-pump check
    PreOpPumpFailed,    // process data crash during 4000-cycle pre-OP pump
    OPTransitionFailed, // slaves did not reach OP after 5000-cycle pump
    SEHException,       // SEH access violation caught by safeCall; exCode populated
};

struct InitResult
{
    bool        ok          = false;
    InitError   error       = InitError::None;
    std::string detail;               // human-readable; also written to m_lastError
    int         failedSlave = -1;     // 1-based slave index; -1 if all/unknown
    uint32_t    exCode      = 0;      // non-zero if SEH caught

    // Implicit bool conversion for backward-compatible call sites
    operator bool() const { return ok; }

    static InitResult success()
    {
        InitResult r;
        r.ok = true;
        return r;
    }

    static InitResult fail(InitError e, std::string msg, int slave = -1, uint32_t ex = 0)
    {
        InitResult r;
        r.ok          = false;
        r.error       = e;
        r.detail      = std::move(msg);
        r.failedSlave = slave;
        r.exCode      = ex;
        return r;
    }
};

class SdoWorker;   // non-RT, cycle-paced CoE SDO worker (see SdoWorker.h)

class EtherCATMaster
{
public:
    EtherCATMaster();
    ~EtherCATMaster();

    // Full init sequence in one call - use this from all code paths
    // (web UI, Qt button, tests). Calls applyConfig(), initialize(),
    // configurePDOs(), enterOperational() in order with correct error
    // handling. Returns InitResult; operator bool() allows legacy bool checks.
    InitResult initializeAndEnterOp(const std::string& nicName);

    // Apply config values before initialize() - called by initializeAndEnterOp().
    // Also called at startup in main.cpp so defaults are correct from the start.
    void applyConfig(const AppConfig& cfg);

    // Low-level steps exposed for MainWindow backward compat - prefer
    // initializeAndEnterOp() for all new code.
    InitResult initialize(const std::string& nicName);
    InitResult configurePDOs();
    InitResult enterOperational();
    void shutdown();
    int  sendReceive();

    A6Drive* getDrive(int index);
    int      getDriveCount() const { return static_cast<int>(m_drives.size()); }
    int      getSlaveCount() const { return m_slaveCount; }

    void enableAllDrives();
    void disableAllDrives();
    void resetAllFaults();

    ECState getMasterState()  const { return static_cast<ECState>(m_masterState.load(std::memory_order_acquire)); }
    bool    isOperational()   const { return static_cast<ECState>(m_masterState.load(std::memory_order_acquire)) == ECState::Op; }
    bool    isInitialized()   const { return m_initialized; }
    bool    isSimulation()    const { return m_simulationMode; }
    // DC reference-clock phase within the cycle (ns), updated each sendReceive.
    // Walks if the loop is not DC-locked; ~constant if it is. Passive diagnostic.
    int64_t getDcPhaseNs()    const { return m_dcPhaseNs.load(std::memory_order_relaxed); }
    void    setSimulationMode(bool sim) { m_simulationMode = sim; }

    std::string getLastError() const { return m_lastError; }

    ecx_contextt* getContext() const
    {
        if (m_simulationMode || !m_initialized) return nullptr;
        return static_cast<ecx_contextt*>(m_ctx);
    }

    int  getExpectedWKC()  const { return m_expectedWKC; }
    bool hasPumpCrashed()  const { return m_pumpCrashed.load(); }

    // Test-only wrappers - expose drainElistImpl.
    // drainElistForTest uses the production cap (100); drainElistForTestCapped
    // accepts a custom cap so tests can trigger the bailout path using a real
    // 64-entry ring (push EC_MAXELIST entries, then drain with max < 64).
    static void drainElistForTest(ecx_contextt* ctx, const char* location);
    static void drainElistForTestCapped(ecx_contextt* ctx, const char* location, int max);

    // Test-only wrappers - expose checkSlaveErrorStateCached and dumpSlaveState.
    static bool checkSlaveErrorStateForTest(ecx_contextt* ctx, int slaveCount, const char* location,
                                             int& failedSlaveOut, std::string& detailOut);
    static void dumpSlaveStateForTest(ecx_contextt* ctx, int slaveCount, const char* location);

    // DS402 / CANopen error-code description lookup.
    // Public-static so tests can verify the table; also called internally by
    // readDriveFaultHistory. Returns "unknown" for codes not in the table.
    static const char* ds402ErrorCodeString(uint16_t code);

    // Test-only wrapper for readDriveFaultHistory state-gating logic.
    // Exposes the readback path so unit tests can verify it skips correctly
    // on slaves whose state is below PRE_OP (where SDO reads would risk
    // triggering the SDO crash family).
    void readDriveFaultHistoryForTest(uint16_t slaveIdx) { readDriveFaultHistory(slaveIdx); }

    void setDriveConfigs(const std::vector<DriveConfig>& configs);
    void setControlLoopHz(int hz)             { m_controlLoopHz = hz; }
    void setDcSyncOffsetNs(int offsetNs)      { m_dcSyncOffsetNs = offsetNs; }
    // DC phase-lock compensator parameters (default OFF). Applied
    // to the background pump's cadence; the control loop reads them from config
    // directly. Set from config before initializeAndEnterOp().
    void setDcPhaseLock(bool en, double kp, double ki, int maxTrimNs)
    {
        m_dcPhaseLockEnabled = en; m_dcPhaseLockKp = kp;
        m_dcPhaseLockKi = ki; m_dcPhaseLockMaxTrimNs = maxTrimNs;
    }
    void setPdoWatchdogMs(int ms)             { m_pdoWatchdogMs = ms; }
    void setEnableCapabilityScan(bool en)     { m_enableCapabilityScan = en; }

    // IGBT temperature round-robin poll interval (seconds PER DRIVE).
    // 0 (default) DISABLES the poll - the worker thread still runs but issues no SDO,
    // so the bus is untouched until this is set > 0 (after the byte-identical trust
    // gate passes). Set from config before initializeAndEnterOp().
    void setTempPollSec(double s)             { m_tempPollSec = s; }
    SdoWorker* sdoWorker() const              { return m_sdoWorker.get(); }   // null until ctor runs
    // Stop the SDO worker's thread. Called at the TOP of a deinit sequence,
    // while the control loop still runs: an in-flight transfer completes
    // normally (the loop's mailbox handler is still servicing it) and the
    // entire teardown -- park, seat, OP->INIT walk -- is then guaranteed
    // mailbox-quiet. Idempotent; shutdown()'s own stop stays as a backstop.
    void stopSdoWorker();

    // Returns true if an init sequence is currently in progress.
    // Use to prevent concurrent init from Qt button + web API.
    bool isInitializing() const { return m_initInProgress.load(); }

    // Called by ControlLoop to claim/release SOEM ownership.
    void setRtLoopActive(bool active) { m_rtLoopActive.store(active, std::memory_order_release); }
    // True while the RT control loop owns SOEM -- i.e. while processCyclicMailbox() is being
    // driven. The SdoWorker gates on this: its cyclic-mailbox SDO is only serviced when the
    // loop runs (the background pump does NOT service the mailbox). PREOP provisioning is a
    // separate direct-SDO path and does not use this.
    bool isRtLoopActive() const { return m_rtLoopActive.load(std::memory_order_acquire); }

    // SdoWorker cross-actor serialisation (full contract in
    // SdoWorker.h). sdoTransferMutex is held for a WHOLE SDO transfer by any SDO actor
    // (the worker, the recovery thread's fault-history read, PREOP provisioning) so
    // there is one CoE transaction per slave at a time. The worker uses SOEM's native
    // cyclic SDO path (ecx_SDOread/write) and deliberately does NOT take
    // m_soemAccessMutex - that would deadlock the RT cyclic handler it waits on.
    std::mutex& sdoTransferMutex()         { return m_sdoTransferMutex; }

    // Observable by tests - true only while T5 (pump thread) is running.
    bool isPumpActive() const { return m_pumpActive.load(std::memory_order_acquire); }

    void stopPump();
    void startPump();

    // Cyclic mailbox handler integration. After SAFE-OP is reached
    // and drive modes are set, slaves with CoE support are added to SOEM's
    // cyclic mailbox queue. processCyclicMailbox() runs from the RT control
    // loop (post-sendReceive) and drains queued mailbox ops up to `limit`
    // per call. This enables real-time emergency message reception from
    // drives without blocking the frame path.
    int  processCyclicMailbox(int limit);

    // Mailbox-work hint. The SdoWorker brackets each
    // transfer with begin/end; while the count is nonzero processCyclicMailbox
    // runs every cycle (transfers complete at full speed), otherwise it drops
    // to 1/16 duty so the frame path skips the mutex + ecx_mbxhandler call on
    // idle cycles (the overwhelming majority).
    void beginMailboxWork() { m_mbxWorkPending.fetch_add(1, std::memory_order_acq_rel); }
    void endMailboxWork()   { m_mbxWorkPending.fetch_sub(1, std::memory_order_acq_rel); }

    // Deinit-seat bus guard: one broadcast state read;
    // copies each slave's AL state into out[0..n-1] (slave 1 -> out[0]).
    // Returns the number of slaves reported, 0 on failure/sim.
    int probeSlaveALStates(uint16_t* out, int maxSlaves);

    // Canonical recovery on a dedicated thread. The RT control loop
    // signals via signalRecoveryNeeded() when WkcMonitor detects sustained
    // errors; the recovery thread wakes, walks each non-OP slave through the
    // canonical 4-branch decision tree (SafeOp+Error → ACK; SafeOp → request
    // OP; valid state → ecx_reconfig_slave; otherwise mark lost), and brings
    // the slave back to OP.
    void signalRecoveryNeeded();
    bool isRecoveryThreadRunning() const { return m_recoveryThreadRunning.load(); }

    // Request a one-shot SDO read of 0x203F (precise panel/Er fault code) for
    // a faulted drive. RT-safe (single fetch_or); the blocking mailbox read
    // itself runs on the recovery thread's 10ms tick. One transaction per
    // fault event -- NOT a poll (sustained SDO polling destabilises DC sync
    // on Windows; see the temperature-poll history).
    void requestPanelCodeRead(int driveIndex);

    // Re-apply PP profile SDOs (0x6081/6083/6084) for a slave that
    // has been recovered via ecx_recover_slave(). SDO values are recalculated
    // from the stored DriveConfig. No-op for CSP/torque drives.
    void reapplyPPProfile(int slaveIdx);

    // State/error notification callbacks. Register before initialize().
    void setOnMasterStateChanged(std::function<void(ECState)> cb) { m_onMasterStateChanged = std::move(cb); }
    void setOnSlaveError(std::function<void(int, const std::string&)> cb) { m_onSlaveError = std::move(cb); }

    // Test-only: force a specific InitResult from enterOperational() in simulation mode.
    // Only takes effect when m_simulationMode = true; ignored on hardware paths.
    void setSimulatedStageError(InitResult r) { m_simulatedStageError = std::move(r); }

private:
    bool   m_initialized = false;
    bool   m_simulationMode = false;
    bool   m_ecxOpen = false;          // true after ecx_init(), false after ecx_close()
    std::atomic<bool> m_initInProgress{false};  // guard against concurrent Qt+web init
    int    m_controlLoopHz = 1000;
    std::string m_resolvedAdapterName;  // pcap device path matched in initialize(), used by retries
    int    m_dcSyncOffsetNs = 0;
    bool   m_dcPhaseLockEnabled   = false;   // pump DC phase-lock (default OFF)
    double m_dcPhaseLockKp        = 2.5;
    double m_dcPhaseLockKi        = 1.6;
    int    m_dcPhaseLockMaxTrimNs = 10000;
    int    m_pdoWatchdogMs  = 100;
    bool   m_enableCapabilityScan = false;    // off by default -- 24 SDO reads per drive
    int    m_commandSyncCycles      = 10;     // cycles to hold at 0x07 before 0x0F
    int    m_sync0RecycleRounds     = 2;      // pre-OP wedged-SYNC0 recycle rounds (0 = off)
    int    m_wkcValidationCycles    = 50;     // post-OP WKC validation window
    double m_wkcValidationThreshold = 0.9;   // fraction of cycles requiring WKC==expected

    alignas(64) char m_iomap[SOEM_IOMAP_SIZE] = {};

    int              m_slaveCount = 0;
    std::atomic<int> m_masterState{static_cast<int>(ECState::None)};  // atomic - read by T1/T2, written by T8
    std::atomic<int64_t> m_dcPhaseNs{0};   // DC reference phase within cycle (ns), passive diagnostic
    std::string      m_lastError;
    int     m_expectedWKC = 0;

    void* m_ctx = nullptr;

    std::optional<InitResult> m_simulatedStageError;  // test injection; see setSimulatedStageError

    bool configureDC();
    void assignPDOPointers();
    void checkSlaveStates();
    void setMasterState(ECState state);
    InitResult tryInitOnce(ecx_contextt* ctx);      // single attempt, called by retry loop
    InitResult tryInitOnceBody(ecx_contextt* ctx);  // body wrapped by safeCall in tryInitOnce()
    void applyPPProfile(ecx_contextt* ctx, int slaveIdx, const DriveConfig& cfg);
    InitResult enterOperationalBody();              // body wrapped by safeCall in enterOperational()

    // Pump dispatcher - bridge + pump thread always run on a
    // persistent thread regardless of which thread calls startPump().
    void startPumpDispatchThread();  // start (or restart after shutdown) the dispatch thread
    void startPumpBody();            // bridge + m_pumpThread launch; called only by dispatch thread

    // Init stage methods.
    // discoverAndPrepareSlaves + stagePreOpSettle + stagePDOConfig + stageDCArm live in tryInitOnceBody().
    // stageSafeOpEntry + stagePreOpPump + stageOPTransition live in enterOperationalBody().
    InitResult discoverAndPrepareSlaves(ecx_contextt* ctx, int& outTimeoutMs); // steps 1–3: state clear, PreOp broadcast, mailbox ping
    InitResult stagePreOpSettle(ecx_contextt* ctx, int timeoutMs);             // polling loop until all slaves stably in PreOp
    InitResult stagePDOConfig(ecx_contextt* ctx);                              // torque RPDO + DC config/hook + AL check + configMap
    InitResult stageDCArm(ecx_contextt* ctx);                                  // verify PO2SO hook result; re-arm SYNC0 if marginNs low
    InitResult stageSafeOpEntry(ecx_contextt* ctx);                            // watchdog config + drive mode SDOs in SafeOp
    InitResult stagePreOpPump(ecx_contextt* ctx);                              // SDO finalisation + 4000-cycle pre-OP pump
    InitResult stageOPTransition(ecx_contextt* ctx);                           // OP request + 5000-cycle pump + bridge + pump thread
    InitResult verifyOperationalWKC(ecx_contextt* ctx);                        // 50-cycle WKC validation after OP confirmed
    void runCapabilityScan(ecx_contextt* ctx);                                 // optional diagnostic SDO reads
    void logOPFailureDiagnostics(ecx_contextt* ctx);                           // per-slave ALState/ALCode/dcAct on OP failure
    void runOPBridge(ecx_contextt* ctx, int64_t ticksPer2Ms);                  // 10-frame Npcap thread warm-up before pump thread
    InitResult pingMailbox(ecx_contextt* ctx);                                 // step 3: mailbox alive check (5 retries × 100ms)

    // Initialize SOEM cyclic mailbox queue and register CoE slaves.
    // Called once from enterOperationalBody() after stageSafeOpEntry succeeds.
    // After this, processCyclicMailbox() can be called from the RT loop to
    // drain queued mailbox ops (emergency frames etc.) without blocking.
    void initCyclicMailboxHandler(ecx_contextt* ctx);
    bool m_mbxHandlerInit = false;                                             // true after initCyclicMailboxHandler() runs

    // Recovery thread lifecycle.
    void startRecoveryThread();
    void stopRecoveryThread();
    void recoveryThreadMain();
    void doRecoveryScan();
    void noteReconfigFailure(int slave, int rc);

    // Per-slave reconfig attempt cap. Indexed by slave
    // number (element 0 unused); sized in startRecoveryThread(). Touched only
    // on the recovery thread - no atomics needed. Reset per slave on
    // successful reconfig and on address recovery (slave power-cycled back).
    static constexpr int kMaxRecoveryReconfigAttempts = 5;
    std::vector<int> m_recoveryReconfigFails;

    // Branch-4 double-confirm: state==NONE can mean "slave gone" OR "the
    // state-read datagram itself died in a frame-loss burst" - a single burst
    // can mark every healthy slave lost in one scan. Mark-lost + input
    // zeroing + mailbox LOST now require TWO consecutive scans agreeing; any
    // healthy read clears the pending flag. A genuinely dropped slave is just
    // as dropped one scan later, so the delay costs nothing real.
    std::vector<uint8_t> m_recoveryLostPending;

    // Read drive fault history via SDO before recovery acts.
    // Reads 0x603F (current error), 0x1001 (error register), 0x1003 (error
    // history list, capped at 5 entries). Gated on state >= PRE_OP so we
    // don't trigger the SDO crash family on slaves that have dropped.
    void readDriveFaultHistory(uint16_t slaveIdx);

    // Mailbox-work hint for the processCyclicMailbox idle fast-path.
    // m_mbxWorkPending is cross-thread (SdoWorker writes, RT loop reads);
    // m_mbxIdleTick is touched only inside processCyclicMailbox (RT/pump/seat
    // callers are serialized by design), so it needs no atomicity.
    std::atomic<int>  m_mbxWorkPending{0};
    uint32_t          m_mbxIdleTick = 0;

    std::thread       m_recoveryThread;
    std::atomic<bool> m_needsRecovery{false};
    std::atomic<bool> m_recoveryStop{false};
    std::atomic<uint32_t> m_panelReadMask{0};   // bit n = read 0x203F from drive n
    std::atomic<bool> m_recoveryThreadRunning{false};

    // Serialises SOEM port access between the RT control loop / pump
    // thread (sendReceive, processCyclicMailbox) and the recovery thread
    // (readstate, writestate, dcsync0, single FPRD). Held only for short
    // operations - ecx_reconfig_slave (~600ms) and ecx_recover_slave (~6ms)
    // run WITHOUT this mutex; SOEM's internal port mutex handles wire-level
    // serialisation. See doRecoveryScan() for rationale.
    //
    // KNOWN LIMITATION - priority inversion: Windows std::mutex is implemented
    // over SRWLock and does NOT do priority inheritance. If the RT thread
    // (TIME_CRITICAL) blocks on this mutex while the recovery thread
    // (ABOVE_NORMAL) holds it, RT effectively waits at the holder's priority.
    // With current short-hold windows (microseconds) this is not catastrophic,
    // but if hardware testing shows RT cycle jitter correlating with recovery
    // activity, upgrade to a PI-capable primitive: CRITICAL_SECTION (via
    // InitializeCriticalSectionAndSpinCount) on Windows, or a custom spinlock
    // for the hot paths.
    std::mutex m_soemAccessMutex;

    // Whole-transfer serialisation for the SdoWorker
    // (see sdoTransferMutex() accessor above).
    std::mutex                  m_sdoTransferMutex;
    std::unique_ptr<SdoWorker>  m_sdoWorker;          // constructed in ctor; started after OP
    double                      m_tempPollSec = 0.0;  // 0 = temp poll disabled (safe default)

    std::vector<std::unique_ptr<A6Drive>> m_drives;
    std::vector<DriveConfig>              m_driveConfigs;

    std::thread m_pumpThread;
    std::atomic<bool> m_pumpActive{ false };
    std::atomic<bool> m_pumpCrashed{ false };

    std::thread              m_pumpDispatchThread;
    std::mutex               m_pumpDispatchMu;
    std::condition_variable  m_pumpDispatchCv;
    bool                     m_pumpDispatchReq  = false;
    bool                     m_pumpDispatchStop = false;

    // Set true by startPump() before dispatch notify, cleared by T4 after
    // startPumpBody() returns (or on dispatch-thread stop). stopPump() spins
    // on this flag so it cannot return while T4 is mid-startPumpBody().
    std::atomic<bool> m_dispatchBusy{false};

    // Set true by ControlLoop before first sendReceive(), cleared before
    // the run-exit startPump() call. startPumpBody() bails if true, preventing
    // T5 launch while T6 owns SOEM.
    std::atomic<bool> m_rtLoopActive{false};

    std::function<void(ECState)>                    m_onMasterStateChanged;
    std::function<void(int, const std::string&)>    m_onSlaveError;
};
