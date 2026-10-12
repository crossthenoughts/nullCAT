// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// TelemetryRecorder - every telemetry datagram as it arrived, to a file,
// so a session can be replayed exactly (tuning, tests).
//
// The receive path runs on the RT loop: there the recorder only copies the
// datagram into a fixed ring of slots allocated once (no lock, no heap, no
// I/O; a full ring drops the datagram and counts it). A writer thread of
// its own, at normal priority, empties the ring to disk. Off, the receive
// path pays one atomic load.
//
// File: one header line, then one line per datagram: milliseconds since
// the recording started, a tab, the datagram as received (its own line
// ending dropped).
// ============================================================

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>

class TelemetryRecorder
{
public:
    static constexpr uint32_t kSlots     = 512;            // ~2.5 s of telemetry at 200 datagrams/s
    static constexpr int      kSlotBytes = 4096;           // the receive buffer's size
    static constexpr uint64_t kMaxBytes  = 2ull << 30;     // a recording stops itself at 2 GB
    static constexpr uint64_t kMinFree   = 1ull << 30;     // ...will not start with under 1 GB free
    static constexpr uint64_t kReserve   = 512ull << 20;   // ...and stops itself sooner to leave this free

    TelemetryRecorder() : m_slots(new Slot[kSlots]) { resetRing(); }
    ~TelemetryRecorder() { stop(); delete[] m_slots; }
    TelemetryRecorder(const TelemetryRecorder&) = delete;
    TelemetryRecorder& operator=(const TelemetryRecorder&) = delete;

    // Web thread: a new file in dir (created if missing) and the writer
    // started. False with the reason when it cannot.
    bool start(const std::string& dir, std::string& why)
    {
        std::lock_guard<std::mutex> lk(m_mx);
        if (m_run.load() && m_on.load()) { why = "already recording"; return false; }
        stopLocked();   // one that stopped itself at the cap
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        if (ec) { why = "cannot create " + dir + ": " + ec.message(); return false; }
        const auto sp = std::filesystem::space(dir, ec);
        if (ec) { why = "cannot read the free space in " + dir + ": " + ec.message(); return false; }
        if (sp.available < kMinFree) { why = "under 1 GB free in " + dir; return false; }
        m_cap = std::min<uint64_t>(kMaxBytes, static_cast<uint64_t>(sp.available) - kReserve);
        char stamp[32];
        const std::time_t now = std::time(nullptr);
        std::tm tmv{};
#ifdef _WIN32
        localtime_s(&tmv, &now);
#else
        localtime_r(&now, &tmv);
#endif
        std::strftime(stamp, sizeof stamp, "rec-%Y%m%d-%H%M%S", &tmv);
        // Never over an earlier recording (a stop and a start in the same
        // second): -2, -3, ... after the time.
        std::string path;
        for (int n = 1; n < 100 && path.empty(); ++n)
        {
            const std::string name = std::string(stamp) + (n > 1 ? "-" + std::to_string(n) : std::string()) + ".ncr";
            const std::string p = (std::filesystem::path(dir) / name).string();
            if (!std::filesystem::exists(p, ec)) path = p;
        }
        if (path.empty()) { why = "no free file name in " + dir; return false; }
        m_file = std::fopen(path.c_str(), "wb");
        if (!m_file) { why = "cannot open " + path; return false; }
        std::fputs("# nullCAT telemetry recording 1: ms since start <TAB> the datagram as received\n", m_file);
        m_path = path; m_capped.store(false); m_failed.store(false);
        m_bytes = m_packets = 0; m_dropped.store(0);
        resetRing();   // stopLocked() waited out any producer mid-push
        m_t0 = nowMs(); m_startMs = m_t0;
        m_run.store(true);
        try { m_writer = std::thread([this] { writerLoop(); }); }
        catch (...)
        {
            m_run.store(false);
            std::fclose(m_file); m_file = nullptr;
            why = "cannot start the writer thread";
            return false;
        }
        m_on.store(true);
        return true;
    }

    // Web thread: stop taking datagrams, write what is in the ring, close.
    void stop()
    {
        std::lock_guard<std::mutex> lk(m_mx);
        stopLocked();
    }

    // Receive path (RT): copy the datagram, never block. Off: one load. On:
    // counted in flight around the copy, so a stop waits it out before the
    // ring can be reset under it.
    void push(const char* data, int len)
    {
        if (!m_on.load(std::memory_order_relaxed) || len <= 0) return;
        m_inflight.fetch_add(1);
        if (m_on.load()) pushOn(data, len);
        m_inflight.fetch_sub(1);
    }

    struct Status
    {
        bool on = false;
        std::string file, note;
        uint64_t bytes = 0, packets = 0, dropped = 0;
        double sec = 0.0;
    };
    Status status() const
    {
        std::lock_guard<std::mutex> lk(m_mx);
        Status s;
        s.on = m_on.load();
        s.note = m_failed.load() ? "stopped: the file could not be written (disk full?)"
               : m_capped.load() ? "stopped at the size cap (2 GB, or to leave 512 MB free)" : "";
        s.file = m_path.empty() ? std::string() : std::filesystem::path(m_path).filename().string();
        s.bytes = m_bytes.load(); s.packets = m_packets.load(); s.dropped = m_dropped.load();
        s.sec = s.on ? (nowMs() - m_startMs) / 1000.0 : 0.0;
        return s;
    }

private:
    void pushOn(const char* data, int len)
    {
        const int n = (len < kSlotBytes) ? len : kSlotBytes;
        uint32_t pos = m_enq.load(std::memory_order_relaxed);
        for (int tries = 0; tries < 8; ++tries)
        {
            Slot& s = m_slots[pos % kSlots];
            const uint32_t seq = s.seq.load(std::memory_order_acquire);
            const int32_t dif = static_cast<int32_t>(seq - pos);
            if (dif == 0)
            {
                if (m_enq.compare_exchange_weak(pos, pos + 1, std::memory_order_relaxed))
                {
                    s.tMs = nowMs() - m_t0;
                    s.len = n;
                    std::memcpy(s.data, data, static_cast<size_t>(n));
                    s.seq.store(pos + 1, std::memory_order_release);
                    return;
                }
            }
            else if (dif < 0) break;   // full: the writer is behind
            else pos = m_enq.load(std::memory_order_relaxed);
        }
        m_dropped.fetch_add(1, std::memory_order_relaxed);
    }

    struct Slot
    {
        std::atomic<uint32_t> seq{0};
        int32_t len = 0;
        int64_t tMs = 0;
        char    data[kSlotBytes];
    };

    static int64_t nowMs()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    void stopLocked()
    {
        m_on.store(false);
        // A producer that saw it on finishes its copy first (microseconds):
        // the ring is drained and later reset under no one.
        while (m_inflight.load() != 0) std::this_thread::yield();
        if (!m_run.load()) return;
        m_run.store(false);
        if (m_writer.joinable()) m_writer.join();
        if (m_file) { std::fclose(m_file); m_file = nullptr; }
    }
    void resetRing()
    {
        for (uint32_t i = 0; i < kSlots; ++i) m_slots[i].seq.store(i, std::memory_order_relaxed);
        m_enq.store(0); m_deq = 0;
    }

    // Writer thread: everything ready in the ring, in order, to the file.
    void writerLoop()
    {
        int64_t lastFlush = nowMs();
        for (;;)
        {
            const bool running = m_run.load();
            bool any = false;
            for (;;)
            {
                Slot& s = m_slots[m_deq % kSlots];
                if (static_cast<int32_t>(s.seq.load(std::memory_order_acquire) - (m_deq + 1)) < 0) break;
                int len = s.len;
                while (len > 0 && (s.data[len - 1] == '\n' || s.data[len - 1] == '\r')) --len;
                const int head = std::fprintf(m_file, "%lld\t", static_cast<long long>(s.tMs));
                std::fwrite(s.data, 1, static_cast<size_t>(len), m_file);
                std::fputc('\n', m_file);
                m_bytes += static_cast<uint64_t>((head > 0 ? head : 0) + len + 1);
                ++m_packets;
                s.seq.store(m_deq + kSlots, std::memory_order_release);
                ++m_deq; any = true;
            }
            // A write that failed (the disk full) stops the recording with
            // a note rather than keep counting bytes that never landed.
            if (std::ferror(m_file) && m_on.load())
            {
                m_on.store(false);
                m_failed.store(true);
            }
            if (m_bytes.load() >= m_cap && m_on.load())
            {
                m_on.store(false);
                m_capped.store(true);
            }
            if (!running) break;   // stop(): the ring is drained above
            const int64_t now = nowMs();
            if (now - lastFlush >= 1000) { std::fflush(m_file); lastFlush = now; }
            if (!any) std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        std::fflush(m_file);
    }

    Slot* m_slots;
    // The producers' line (the receive path) apart from the writer's.
    alignas(64) std::atomic<uint32_t> m_enq{0};
    std::atomic<bool> m_on{false};
    std::atomic<int>  m_inflight{0};
    std::atomic<uint64_t> m_dropped{0};
    int64_t m_t0 = 0;
    alignas(64) uint32_t m_deq = 0;           // writer thread only
    std::atomic<bool> m_run{false}, m_capped{false}, m_failed{false};
    std::atomic<uint64_t> m_bytes{0}, m_packets{0};
    uint64_t m_cap = kMaxBytes;
    int64_t m_startMs = 0;
    std::FILE* m_file = nullptr;
    std::thread m_writer;
    mutable std::mutex m_mx;                  // start / stop / status
    std::string m_path;
};
