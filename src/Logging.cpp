// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// Logging.cpp — RT-safe logging implementation.
// ============================================================
#include "Logging.h"

#include <iostream>
#include <chrono>
#include <ctime>
#include <cstdio>
#include <cstdarg>
#include <filesystem>

// ---- Timestamp helper ----
static std::string currentTimestamp()
{
    using namespace std::chrono;
    auto now    = system_clock::now();
    auto tt     = system_clock::to_time_t(now);
    auto us     = duration_cast<microseconds>(now.time_since_epoch()).count() % 1000000;
    char tbuf[24];
    // localtime is not thread-safe on all platforms; use localtime_s on Windows
#ifdef _WIN32
    struct tm tm_info;
    localtime_s(&tm_info, &tt);
    std::strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", &tm_info);
#else
    struct tm tm_info;
    localtime_r(&tt, &tm_info);
    std::strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", &tm_info);
#endif
    char result[36];
    snprintf(result, sizeof(result), "%s.%06lld", tbuf, static_cast<long long>(us));
    return result;
}

// ============================================================

Logger& Logger::instance()
{
    static Logger inst;
    return inst;
}

Logger::Logger()
    : m_toConsole(true)
    , m_minLevel(LogLevel::LVL_DEBUG)
{
}

Logger::~Logger()
{
    if (m_drainRunning.exchange(false))
    {
        if (m_drainThread.joinable())
            m_drainThread.join();
    }

    // Final drain — file only, callback may be gone
    {
        LogEntry entry;
        std::lock_guard<std::mutex> lock(m_mutex);
        while (m_rtQueue.pop(entry))
        {
            if (m_file.is_open())
            {
                m_file << "[" << currentTimestamp() << "] ["
                       << levelToString(entry.level) << "] "
                       << entry.msg << "\n";
            }
        }
        if (m_file.is_open())
        {
            m_file.flush();
            m_file.close();
        }
        if (m_diagFile.is_open())
        {
            m_diagFile.flush();
            m_diagFile.close();
        }
    }
}

void Logger::init(const std::string& logFilePath, bool toConsole)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_toConsole = toConsole;

    // Create parent directory if needed
    try {
        auto parent = std::filesystem::path(logFilePath).parent_path();
        if (!parent.empty())
            std::filesystem::create_directories(parent);
    } catch (...) {}

    if (m_file.is_open())
    {
        m_file.flush();
        m_file.close();
    }

    m_filePath = logFilePath;
    try { m_fileBytes = std::filesystem::exists(logFilePath)
              ? std::filesystem::file_size(logFilePath) : 0; }
    catch (...) { m_fileBytes = 0; }

    m_file.open(logFilePath, std::ios::app);
    if (m_file.is_open())
    {
        m_file << "\n=== Log started: " << currentTimestamp() << " ===\n";
        m_file.flush();
        rotateIfNeeded(m_file, m_filePath, m_fileBytes);   // inherited oversize file rotates immediately
    }
}

void Logger::initDiag(const std::string& diagFilePath)
{
    try {
        auto parent = std::filesystem::path(diagFilePath).parent_path();
        if (!parent.empty())
            std::filesystem::create_directories(parent);
    } catch (...) {}

    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_diagFile.is_open())
    {
        m_diagFile.flush();
        m_diagFile.close();
    }
    m_diagPath = diagFilePath;
    try { m_diagBytes = std::filesystem::exists(diagFilePath)
              ? std::filesystem::file_size(diagFilePath) : 0; }
    catch (...) { m_diagBytes = 0; }

    m_diagFile.open(diagFilePath, std::ios::app);
    if (m_diagFile.is_open())
    {
        m_diagFile << "\n=== SOEM diag started: " << currentTimestamp() << " ===\n";
        m_diagFile.flush();
    }
}

void Logger::logDiag(const std::string& line)
{
    // Short-circuit when disabled: avoids the heap alloc + mutex acquire
    // + file write entirely for the cost of one atomic load.
    // NON-RT threads only (init/recovery/UI). RT-thread diag goes through
    // RT_DIAG -> SPSC ring -> writeDiagLine on the drain thread.
    if (!m_diagEnabled.load(std::memory_order_acquire))
        return;
    writeDiagLine(line.c_str());
}

void Logger::writeDiagLine(const char* msg)
{
    // Shared diag-file writer (logDiag + drained LVL_DIAG ring entries).
    // Timestamp is stamped HERE — for ring entries that is drain time, a few
    // ms after the RT push, same latency semantics as RT_LOG lines.
    std::string full = currentTimestamp() + " | " + msg;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_diagFile.is_open())
    {
        // No per-line flush while the RT path is armed: a flush here holds
        // the shared mutex through file I/O and contributes to RT
        // cycle-deadline misses. While armed, the drain thread flushes ~1s;
        // when NOT armed (init, idle) flush per line so cold-start evidence
        // is on disk the moment it is written.
        m_diagFile << full << "\n";
        m_diagBytes += full.size() + 1;
        rotateIfNeeded(m_diagFile, m_diagPath, m_diagBytes);
        if (m_rtThreadId.load(std::memory_order_relaxed) == std::thread::id{})
            m_diagFile.flush();
    }
}

LogLevel Logger::parseLevel(const std::string& s)
{
    // Case-insensitive match. Unrecognised -> LVL_DEBUG (most permissive).
    std::string lower;
    lower.reserve(s.size());
    for (char c : s) lower += static_cast<char>(::tolower(static_cast<unsigned char>(c)));
    if (lower == "critical") return LogLevel::LVL_CRITICAL;
    if (lower == "error")    return LogLevel::LVL_ERROR;
    if (lower == "warning")  return LogLevel::LVL_WARNING;
    if (lower == "info")     return LogLevel::LVL_INFO;
    if (lower == "debug")    return LogLevel::LVL_DEBUG;
    return LogLevel::LVL_DEBUG;
}

void Logger::setRTThread()
{
    m_rtThreadId = std::this_thread::get_id();

    if (!m_drainRunning.exchange(true))
    {
        m_drainThread = std::thread([this]()
        {
            int sinceFlush = 0;
            while (m_drainRunning.load(std::memory_order_acquire))
            {
                drainQueue();
                // While the RT path is armed, per-line flushing is off (see
                // writeEntry/writeDiagLine); commit the buffered sinks about
                // once a second from here, off the RT thread.
                if (++sinceFlush >= 100)
                {
                    sinceFlush = 0;
                    std::lock_guard<std::mutex> lock(m_mutex);
                    std::cout.flush();
                    if (m_diagFile.is_open()) m_diagFile.flush();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
    }
}

void Logger::clearRTThread()
{
    m_rtThreadId = std::thread::id{};

    if (m_drainRunning.exchange(false))
    {
        if (m_drainThread.joinable())
            m_drainThread.join();
        drainQueue();
        std::lock_guard<std::mutex> lock(m_mutex);
        std::cout.flush();
        if (m_diagFile.is_open()) m_diagFile.flush();
    }
}

void Logger::drainQueue()
{
    LogEntry entry;
    while (m_rtQueue.pop(entry))
    {
        // LVL_DIAG ring entries route to the diag file, not the main log.
        if (entry.level == LogLevel::LVL_DIAG) { writeDiagLine(entry.msg); continue; }
        if (entry.level < m_minLevel.load(std::memory_order_relaxed)) continue;
        writeEntry(entry.level, entry.msg);
    }

    uint32_t dropped = m_droppedRT.load(std::memory_order_relaxed);
    if (dropped > 0)
    {
        uint32_t prev = m_droppedRT.exchange(0, std::memory_order_relaxed);
        if (prev > 0)
            writeEntry(LogLevel::LVL_WARNING,
                "Logger: " + std::to_string(prev) + " RT log entries dropped (queue full)");
    }
}

// Caller holds m_mutex. Never runs on the RT thread (writeEntry/writeDiagLine
// execute on writer or drain threads only). One backup generation: the cap
// bounds disk use at ~2x kLogRotateBytes per stream.
void Logger::rotateIfNeeded(std::ofstream& f, std::string& path, std::uintmax_t& bytes)
{
    if (bytes < kLogRotateBytes || path.empty() || !f.is_open()) return;
    f.flush();
    f.close();
    const std::string backup = path + ".1";
    std::error_code ec;
    std::filesystem::remove(backup, ec);
    std::filesystem::rename(path, backup, ec);   // on failure we truncate below anyway
    f.open(path, std::ios::trunc);
    bytes = 0;
    if (f.is_open())
        f << "=== Log rotated (previous: " << backup << "): "
          << currentTimestamp() << " ===\n";
}

void Logger::writeEntry(LogLevel level, const std::string& message)
{
    std::string ts   = currentTimestamp();
    const char* lvl  = levelToString(level);
    std::string line = "[" + ts + "] [" + lvl + "] " + message;

    {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (m_toConsole)
        {
            if (level >= LogLevel::LVL_WARNING)
                std::cerr << line << "\n";
            else
            {
                std::cout << line << "\n";
                // stdout is block-buffered when piped (systemd journal): an
                // idle process can hold its last INFO lines invisible for
                // hours. Flush per line whenever the RT path is not armed;
                // while armed, the drain thread flushes ~1s instead.
                if (m_rtThreadId.load(std::memory_order_relaxed) == std::thread::id{})
                    std::cout.flush();
            }
        }

        if (m_file.is_open())
        {
            m_file << line << "\n";
            m_file.flush();
            m_fileBytes += line.size() + 1;
            rotateIfNeeded(m_file, m_filePath, m_fileBytes);
        }

        if (m_logCallback)
            m_logCallback(line, static_cast<int>(level));

        // Ring buffer for web UI log panel
        m_ringBuffer.push_back(line);
        if (static_cast<int>(m_ringBuffer.size()) > kRingBufferMax)
            m_ringBuffer.pop_front();
    }
}

void Logger::pushRT(LogLevel level, const char* fmt, ...) noexcept
{
    // LVL_DIAG (5) is above every settable minLevel, so it always passes.
    if (level < m_minLevel.load(std::memory_order_relaxed)) return;

    LogEntry entry;
    entry.level = level;

    va_list ap;
    va_start(ap, fmt);
    // vsnprintf writes at most sizeof(entry.msg)-1 chars + null terminator.
    // Returns the number of chars that WOULD have been written; if >= sizeof(entry.msg),
    // the output was truncated.
    int n = vsnprintf(entry.msg, sizeof(entry.msg), fmt, ap);
    va_end(ap);

    if (n < 0 || n >= (int)sizeof(entry.msg))
    {
        // Overwrite the last 3 payload chars with "..." so truncation is visible.
        entry.msg[sizeof(entry.msg) - 4] = '.';
        entry.msg[sizeof(entry.msg) - 3] = '.';
        entry.msg[sizeof(entry.msg) - 2] = '.';
        entry.msg[sizeof(entry.msg) - 1] = '\0';
    }

    if (!m_rtQueue.push(entry))
        m_droppedRT.fetch_add(1, std::memory_order_relaxed);
}

void Logger::log(LogLevel level, const std::string& message)
{
    if (level < m_minLevel.load(std::memory_order_relaxed)) return;

    // RT path -- lock-free push to the SPSC queue.
    const std::thread::id rtId = m_rtThreadId.load(std::memory_order_relaxed);
    if (std::this_thread::get_id() == rtId && rtId != std::thread::id{})
    {
        LogEntry entry;
        entry.level = level;
        strncpy(entry.msg, message.c_str(), sizeof(entry.msg) - 1);
        entry.msg[sizeof(entry.msg) - 1] = '\0';

        if (!m_rtQueue.push(entry))
            m_droppedRT.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    // Non-RT path: direct write with mutex
    writeEntry(level, message);
}

void Logger::flush()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if (m_file.is_open())     m_file.flush();
    if (m_diagFile.is_open()) m_diagFile.flush();
}

const char* Logger::levelToString(LogLevel level)
{
    switch (level)
    {
        case LogLevel::LVL_DEBUG:    return "DEBUG   ";
        case LogLevel::LVL_INFO:     return "INFO    ";
        case LogLevel::LVL_WARNING:  return "WARNING ";
        case LogLevel::LVL_ERROR:    return "ERROR   ";
        case LogLevel::LVL_CRITICAL: return "CRITICAL";
        default:                     return "UNKNOWN ";
    }
}
