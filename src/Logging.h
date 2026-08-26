// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// Logging.h - RT-safe logging singleton (Qt-free).
//
// Logger is a plain singleton (no QObject, no signals).
//
// The UI receives log lines via a std::function callback
// registered with setLogCallback(). MainWindow registers a
// lambda that marshals to the Qt thread via invokeMethod.
//
// strf() - lightweight printf-style std::string formatter.
// All LOG_* helpers take const std::string& (or const char*
// which converts implicitly).
//
// RT-safe path: push to SPSC queue on the RT thread; a drain
// thread writes/dispatches on the non-RT side.
// ============================================================

#include "SpscQueue.h"

#include <string>
#include <cstdint>
#include <mutex>
#include <fstream>
#include <functional>
#include <thread>
#include <atomic>
#include <deque>
#include <vector>
#include <cstdio>
#include <cstring>

// Prefix all enum values with LVL_ to avoid Windows macro conflicts
enum class LogLevel
{
    LVL_DEBUG    = 0,
    LVL_INFO     = 1,
    LVL_WARNING  = 2,
    LVL_ERROR    = 3,
    LVL_CRITICAL = 4,
    // RT-thread diag lines. Above CRITICAL so minLevel can
    // never filter them (diag has its own gate: m_diagEnabled). The drain
    // thread routes these to the DIAG file, not the main log.
    LVL_DIAG     = 5
};

// Fixed-size log entry for the RT SPSC queue.
// Must be trivially copyable. Message truncated at 479 chars.
struct LogEntry
{
    LogLevel level = LogLevel::LVL_INFO;
    char     msg[479] = {};
};
static_assert(std::is_trivially_copyable<LogEntry>::value,
    "LogEntry must be trivially copyable for SpscQueue");

// ---- strf() - printf-style std::string formatter ----
// Use in LOG_* calls instead of QString("...").arg(...):
//   LOG_INFO(strf("Axis %d pos=%.2f", axisIdx, pos));
template<typename... Args>
inline std::string strf(const char* fmt, Args... args)
{
    char buf[480];
    snprintf(buf, sizeof(buf), fmt, args...);
    return std::string(buf);
}

class Logger
{
public:
    static Logger& instance();

    void init(const std::string& logFilePath, bool toConsole = true);
    void initDiag(const std::string& diagFilePath);
    void logDiag(const std::string& line);

    // Call once at RT thread start to arm the lock-free path.
    void setRTThread();
    // Call at RT thread exit to stop the drain thread cleanly.
    void clearRTThread();

    // Register a callback to receive formatted log lines.
    // Called from the drain thread (non-RT) or directly for non-RT logs.
    // The callback must be thread-safe (or use invokeMethod to marshal).
    // Pass nullptr to clear.
    void setLogCallback(std::function<void(const std::string&, int)> cb)
    {
        std::lock_guard<std::mutex> lk(m_mutex);
        m_logCallback = std::move(cb);
    }

    void log(LogLevel level, const std::string& message);

    void debug   (const std::string& msg) { log(LogLevel::LVL_DEBUG,    msg); }
    void info    (const std::string& msg) { log(LogLevel::LVL_INFO,     msg); }
    void warning (const std::string& msg) { log(LogLevel::LVL_WARNING,  msg); }
    void error   (const std::string& msg) { log(LogLevel::LVL_ERROR,    msg); }
    void critical(const std::string& msg) { log(LogLevel::LVL_CRITICAL, msg); }

    // RT-safe log path: formats directly into a LogEntry char buffer via vsnprintf.
    // Zero heap allocation. No mutex. Pushes to the SPSC ring (same as the RT path
    // in log(), but callable directly without constructing a std::string argument).
    // RT thread only - the SPSC ring has exactly one producer.
    // Messages exceeding 475 chars are truncated and marked with "...".
#if defined(__GNUC__) || defined(__clang__)
    // Non-static member: implicit `this` is arg 1, so fmt is arg 3, varargs arg 4.
    void pushRT(LogLevel level, const char* fmt, ...) noexcept
        __attribute__((format(printf, 3, 4)));
#else
    void pushRT(LogLevel level, const char* fmt, ...) noexcept;
#endif

    void setMinLevel(LogLevel level) { m_minLevel.store(level, std::memory_order_relaxed); }

    // Gate for logDiag(). When false, logDiag() short-circuits - no string
    // allocation, no mutex acquire, no file write - so high-rate DIAG
    // emissions (RTT samples, pump_samples) can be silenced when probing
    // RT-loop jitter. Default true preserves observability.
    void setDiagEnabled(bool enabled) { m_diagEnabled.store(enabled, std::memory_order_release); }
    bool isDiagEnabled() const        { return m_diagEnabled.load(std::memory_order_acquire); }

    // Parse "debug"/"info"/"warning"/"error"/"critical" (case-insensitive)
    // to LogLevel. Returns LVL_DEBUG for unrecognised input. Used by main.cpp
    // to wire config -> Logger at startup.
    static LogLevel parseLevel(const std::string& s);

    // Force both log streams to disk immediately.
    // Call from SEH __except blocks before any heap-allocating operation.
    // Safe to call concurrently with normal log writes (holds m_mutex briefly).
    void flush();

    // Returns up to `n` most recent formatted log lines (newest last).
    // Thread-safe - safe to call from the web server thread.
    std::vector<std::string> getRecentLogs(int n = 200) const
    {
        std::lock_guard<std::mutex> lk(m_mutex);
        int count = static_cast<int>(m_ringBuffer.size());
        int start = std::max(0, count - n);
        return std::vector<std::string>(
            m_ringBuffer.begin() + start, m_ringBuffer.end());
    }

private:
    static constexpr int kRingBufferMax = 500;
    Logger();
    ~Logger();
    Logger(const Logger&)            = delete;
    Logger& operator=(const Logger&) = delete;

    void writeEntry(LogLevel level, const std::string& message);
    void writeDiagLine(const char* msg);   // shared diag-file writer
    void drainQueue();

    mutable std::mutex m_mutex;
    // Size-based rotation (appliance discipline: a debug-on rig must
    // never fill its SD card). At kLogRotateBytes the file is closed,
    // renamed to <name>.1 (replacing any previous .1) and reopened,
    // so the worst case on disk is ~2x the cap per stream. Rotation
    // runs under m_mutex on writer/drain threads only -- never RT.
    static constexpr std::uintmax_t kLogRotateBytes = 64ull * 1024 * 1024;
    std::string    m_filePath;
    std::string    m_diagPath;
    std::uintmax_t m_fileBytes = 0;
    std::uintmax_t m_diagBytes = 0;
    void rotateIfNeeded(std::ofstream& f, std::string& path, std::uintmax_t& bytes);
    std::ofstream m_file;
    std::ofstream m_diagFile;
    std::deque<std::string> m_ringBuffer;  // protected by m_mutex
    bool         m_toConsole = true;
    // Atomic - written by setMinLevel() from any thread, read on the RT
    // thread in pushRT()/log().
    std::atomic<LogLevel> m_minLevel{LogLevel::LVL_DEBUG};

    std::function<void(const std::string&, int)> m_logCallback;

    // RT SPSC queue
    SpscQueue<LogEntry, 1024> m_rtQueue;
    std::thread               m_drainThread;
    std::atomic<bool>         m_drainRunning{false};
    // Atomic - set/cleared by the RT thread lifecycle, compared on every
    // log() call from any thread.
    std::atomic<std::thread::id> m_rtThreadId{std::thread::id{}};
    std::atomic<uint32_t>     m_droppedRT{0};

    // Gate for logDiag (atomic so set/check is cheap from any thread).
    std::atomic<bool>         m_diagEnabled{true};

    static const char* levelToString(LogLevel level);
};

inline void LOG_DEBUG   (const std::string& msg) { Logger::instance().debug(msg);    }
inline void LOG_INFO    (const std::string& msg) { Logger::instance().info(msg);     }
inline void LOG_WARNING (const std::string& msg) { Logger::instance().warning(msg);  }
inline void LOG_ERROR   (const std::string& msg) { Logger::instance().error(msg);    }
inline void LOG_CRITICAL(const std::string& msg) { Logger::instance().critical(msg); }

// RT-safe variants: printf-style, zero heap allocation.
// Use only from the RT thread. See Logger::pushRT() for semantics.
// MSVC: ##__VA_ARGS__ suppresses the trailing comma when no args are supplied.
#define RT_LOG_DEBUG(fmt, ...)   Logger::instance().pushRT(LogLevel::LVL_DEBUG,   fmt, ##__VA_ARGS__)
#define RT_LOG_INFO(fmt, ...)    Logger::instance().pushRT(LogLevel::LVL_INFO,    fmt, ##__VA_ARGS__)
#define RT_LOG_WARNING(fmt, ...) Logger::instance().pushRT(LogLevel::LVL_WARNING, fmt, ##__VA_ARGS__)
#define RT_LOG_ERROR(fmt, ...)   Logger::instance().pushRT(LogLevel::LVL_ERROR,   fmt, ##__VA_ARGS__)

// RT-thread diag lines. logDiag() heap-allocates and takes the
// shared mutex (held through file flushes by the drain thread) - a priority-
// inversion window on the RT thread. RT_DIAG instead rides the same lock-free
// SPSC ring as RT_LOG_*, and gates on the diagEnabled atomic BEFORE formatting
// so a disabled diag costs exactly one atomic load and never pays for string
// formatting. Non-RT threads keep calling logDiag() directly - the mutex is
// fine off the RT path.
#define RT_DIAG(fmt, ...) \
    do { if (Logger::instance().isDiagEnabled()) \
             Logger::instance().pushRT(LogLevel::LVL_DIAG, fmt, ##__VA_ARGS__); } while (0)
