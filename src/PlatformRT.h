// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// PlatformRT.h — platform abstraction for real-time thread
// primitives. Isolates all OS-specific RT code so ControlLoop.cpp
// and EtherCATMaster.cpp compile on Windows and Linux without
// #ifdefs in the logic.
//
// Windows:  QueryPerformanceCounter, timeBeginPeriod,
//           AvSetMmThreadCharacteristics, SetThreadPriority,
//           SetThreadAffinityMask
// Linux:    clock_gettime(CLOCK_MONOTONIC), pthread_setschedparam
//           (SCHED_FIFO), sched_setaffinity, clock_nanosleep
// ============================================================

#include "Logging.h"

#ifdef _WIN32
#  include <windows.h>
#  include <mmsystem.h>
#  include <avrt.h>
#  pragma comment(lib, "winmm.lib")
#else
#  include <pthread.h>
#  include <sched.h>
#  include <sys/mman.h>   // mlockall
#  include <time.h>
#  include <unistd.h>
#  include <cstring>
#endif

#include <cstdint>

namespace PlatformRT
{

// ---- Opaque handle for platform thread state ----
#ifdef _WIN32
struct RtHandle
{
    HANDLE hThread  = nullptr;
    HANDLE hMmTask  = nullptr;
};
#else
struct RtHandle {};
#endif

// ---- Timestamp type ----
#ifdef _WIN32
using Timestamp = LARGE_INTEGER;
#else
using Timestamp = struct timespec;
#endif

// ============================================================
// qpfTicks() — cached QueryPerformanceFrequency (Windows only).
// Queried once; safe because QPC frequency is constant per boot.
// All hot-path timing functions use this instead of calling
// QueryPerformanceFrequency() on every tick.
// ============================================================
#ifdef _WIN32
inline int64_t qpfTicks()
{
    static const int64_t freq = []() -> int64_t {
        LARGE_INTEGER f;
        QueryPerformanceFrequency(&f);
        return f.QuadPart;
    }();
    return freq;
}
#endif

// ============================================================
// threadSetup()
// Elevates RT priority, registers with MMCSS (Windows) or
// sets SCHED_FIFO (Linux), pins to coreIndex, and pre-faults
// 64KB of stack to eliminate first-cycle page-fault spikes.
// Call at the start of the RT thread's run() function.
// ============================================================
inline RtHandle threadSetup(int coreIndex = 3)
{
    RtHandle h;

#ifdef _WIN32
    h.hThread = GetCurrentThread();
    SetThreadPriority(h.hThread, THREAD_PRIORITY_TIME_CRITICAL);

    DWORD taskIndex = 0;
    // MMCSS class "Pro Audio" (not "Games"): designed for sub-5ms audio
    // buffer latency, which matches the 500Hz / 2ms PDO cycle, and avoids
    // sharing the MMCSS budget with SimHub / racing sims (which typically
    // register as Games). AVRT_PRIORITY_CRITICAL is the intra-class priority.
    h.hMmTask = AvSetMmThreadCharacteristics(L"Pro Audio", &taskIndex);
    if (h.hMmTask)
        AvSetMmThreadPriority(h.hMmTask, AVRT_PRIORITY_CRITICAL);
    else
        LOG_WARNING("PlatformRT: AvSetMmThreadCharacteristics(Pro Audio) failed -- timer may degrade in background.");

    DWORD_PTR affinityMask = (1ULL << coreIndex);
    DWORD_PTR prevAffinity = SetThreadAffinityMask(h.hThread, affinityMask);
    if (prevAffinity != 0)
        LOG_INFO(strf("PlatformRT: Thread pinned to core %d (affinity mask=0x%llx)",
            coreIndex, (unsigned long long)affinityMask));
    else
        LOG_WARNING("PlatformRT: Failed to set thread affinity -- running on default cores.");

    // Pre-fault 64KB of stack to eliminate first-cycle page-fault latency spikes.
    // Windows commits stack pages lazily; touching them now pays the fault cost
    // before the RT loop starts rather than during a live EtherCAT frame.
    {
        ULONG stackGuarantee = 64 * 1024;
        SetThreadStackGuarantee(&stackGuarantee);

        static const int kPrefaultBytes = 64 * 1024;
        volatile char stackBuf[kPrefaultBytes];
        for (int i = 0; i < kPrefaultBytes; i += 4096)
            stackBuf[i] = 0;
    }
    LOG_INFO("PlatformRT: Stack pre-faulted (64KB).");

#else
    // SCHED_FIFO requires CAP_SYS_NICE (run as root or set capabilities)
    struct sched_param sp = {};
    sp.sched_priority = 90;  // 1–99; 90 is standard for RT EtherCAT threads
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) == 0)
        LOG_INFO(strf("PlatformRT: Thread set to SCHED_FIFO priority 90, core %d.", coreIndex));
    else
        LOG_WARNING("PlatformRT: pthread_setschedparam failed -- not running as root? RT performance may suffer.");

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(coreIndex, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) == 0)
        LOG_INFO(strf("PlatformRT: Thread pinned to core %d.", coreIndex));
    else
        LOG_WARNING("PlatformRT: pthread_setaffinity_np failed.");

    // Lock the process's memory so the RT loop can never be paged out
    // under memory pressure (swap-thrash on a small-RAM host would
    // otherwise stall live EtherCAT cycles). The memlock rlimits are
    // provisioned by setup.sh + the systemd unit.
    // Process-wide and idempotent; MCL_FUTURE covers later allocations
    // (~tens of MB total on this appliance -- cheap insurance on 2GB).
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == 0)
        LOG_INFO("PlatformRT: mlockall(MCL_CURRENT|MCL_FUTURE) -- process memory locked.");
    else
        LOG_WARNING("PlatformRT: mlockall failed (memlock rlimit?) -- RT memory is swappable.");

    // Pre-fault the RT stack, mirroring the Windows branch: touch the
    // pages before the loop starts so no first-use page fault lands
    // mid-cycle. With mlockall(MCL_FUTURE) above, the touched pages also
    // stay resident.
    {
        constexpr int kPrefaultBytes = 64 * 1024;
        volatile char stackBuf[kPrefaultBytes];
        for (int i = 0; i < kPrefaultBytes; i += 4096)
            stackBuf[i] = 0;
        LOG_INFO("PlatformRT: RT stack pre-faulted (64KB).");
    }
#endif

    return h;
}

// ============================================================
// threadTeardown()
// Reverts MMCSS registration (Windows). No-op on Linux.
// Call at the end of the RT thread's run() function.
// ============================================================
inline void threadTeardown(RtHandle& h)
{
#ifdef _WIN32
    if (h.hMmTask)
    {
        AvRevertMmThreadCharacteristics(h.hMmTask);
        h.hMmTask = nullptr;
    }
#else
    (void)h;
#endif
}

// ============================================================
// timerBegin() / timerEnd()
// Windows: sets system timer to 1ms via timeBeginPeriod.
// This tightens the overshoot window in waitUntil()'s Sleep().
// Linux: no-op (kernel uses high-res timers by default).
// ============================================================
inline void timerBegin()
{
#ifdef _WIN32
    // 1ms system timer resolution — sufficient for the hybrid sleep+busy-wait
    // in waitUntil(). NtSetTimerResolution is deliberately NOT used: it is
    // system-wide and doubles the timer interrupt rate, measurably
    // increasing RT jitter.
    timeBeginPeriod(1);
#endif
}

inline void timerEnd()
{
#ifdef _WIN32
    timeEndPeriod(1);
#endif
}

// ============================================================
// now() — returns the current high-resolution timestamp.
// ============================================================
inline Timestamp now()
{
#ifdef _WIN32
    LARGE_INTEGER t;
    QueryPerformanceCounter(&t);
    return t;
#else
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t;
#endif
}

// ============================================================
// countsPerMicro() — conversion factor from timestamp ticks to µs.
// On Linux this is always 1000.0 (nanoseconds → microseconds).
// ============================================================
inline double countsPerMicro()
{
#ifdef _WIN32
    return static_cast<double>(qpfTicks()) / 1e6;
#else
    return 1000.0;  // timespec.tv_nsec is nanoseconds; 1µs = 1000ns
#endif
}

// ============================================================
// addMicros() — returns a timestamp advanced by periodUs microseconds.
// ============================================================
inline Timestamp addMicros(const Timestamp& t, double periodUs)
{
#ifdef _WIN32
    LARGE_INTEGER result = t;
    result.QuadPart += static_cast<int64_t>(periodUs * (static_cast<double>(qpfTicks()) / 1e6));
    return result;
#else
    int64_t nsAdd = static_cast<int64_t>(periodUs * 1000.0);
    struct timespec result = t;
    result.tv_nsec += nsAdd;
    while (result.tv_nsec >= 1000000000L)
    {
        result.tv_sec  += 1;
        result.tv_nsec -= 1000000000L;
    }
    return result;
#endif
}

// ============================================================
// advancePeriod() — advances a deadline timestamp by exactly
// one period's worth of ticks. Avoids accumulated float error.
// ============================================================
inline void advancePeriod(Timestamp& deadline, int64_t periodCounts)
{
#ifdef _WIN32
    deadline.QuadPart += periodCounts;
#else
    deadline.tv_nsec += static_cast<long>(periodCounts);
    while (deadline.tv_nsec >= 1000000000L)
    {
        deadline.tv_sec  += 1;
        deadline.tv_nsec -= 1000000000L;
    }
#endif
}

// ============================================================
// periodCounts() — converts a cycle time in µs to integer ticks.
// Use with advancePeriod() to avoid float accumulation.
// ============================================================
inline int64_t periodCounts(double cycleTimeUs)
{
#ifdef _WIN32
    return static_cast<int64_t>(cycleTimeUs * (static_cast<double>(qpfTicks()) / 1e6));
#else
    return static_cast<int64_t>(cycleTimeUs * 1000.0);  // µs → ns
#endif
}

// ============================================================
// elapsedMicros() — signed difference: (a - b) in microseconds.
// Positive when a is after b.
// ============================================================
inline double elapsedMicros(const Timestamp& a, const Timestamp& b)
{
#ifdef _WIN32
    return static_cast<double>(a.QuadPart - b.QuadPart) /
           (static_cast<double>(qpfTicks()) / 1e6);
#else
    int64_t nsA = static_cast<int64_t>(a.tv_sec) * 1000000000LL + a.tv_nsec;
    int64_t nsB = static_cast<int64_t>(b.tv_sec) * 1000000000LL + b.tv_nsec;
    return static_cast<double>(nsA - nsB) / 1000.0;
#endif
}

// ============================================================
// waitUntil() — sleeps until near the deadline, then busy-waits
// the final 100µs for precision. Uses cached QPC frequency.
// ============================================================
inline void waitUntil(const Timestamp& deadline)
{
#ifdef _WIN32
    const int64_t freq = qpfTicks();
    const int64_t busyThreshold = static_cast<int64_t>(100.0 * (static_cast<double>(freq) / 1e6));
    LARGE_INTEGER now;
    while (true)
    {
        QueryPerformanceCounter(&now);
        int64_t remaining = deadline.QuadPart - now.QuadPart;
        if (remaining <= 0) break;
        if (remaining > busyThreshold)
        {
            DWORD sleepMs = static_cast<DWORD>(
                (remaining - busyThreshold) / (freq / 1000));
            if (sleepMs > 0) Sleep(sleepMs);
        }
    }
#else
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
#endif
}

// ============================================================
// safeCall() — wraps a function call in a SEH handler on
// Windows, plain call on Linux. Returns true on success.
// exceptionCode is set to the SEH code on Windows if an
// exception fires; always 0 on Linux.
// ============================================================
// __declspec(noinline) prevents MSVC from inlining __try into caller functions
// that have C++ objects (std::string temporaries etc.). Inlining __try into such
// a caller breaks the SEH frame setup (C2712-adjacent behaviour) so the exception
// escapes instead of being caught. Keeping safeCall in its own non-inlined frame
// ensures the __try has proper SEH scope regardless of the caller's local objects.
template<typename Func>
#ifdef _WIN32
__declspec(noinline)
#endif
bool safeCall(Func&& fn, uint32_t* exceptionCode)
{
    *exceptionCode = 0;
#ifdef _WIN32
    __try
    {
        fn();
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        *exceptionCode = static_cast<uint32_t>(GetExceptionCode());
        // Flush log streams before any further operations.
        // SDO crashes (0xc0000005) can corrupt heap state; flushing here
        // ensures diagnostic entries written before the crash reach disk
        // before any subsequent heap operation can fail or hang.
        Logger::instance().flush();
        fflush(stdout);
        fflush(stderr);
        return false;
    }
#else
    fn();
    return true;
#endif
}

} // namespace PlatformRT
