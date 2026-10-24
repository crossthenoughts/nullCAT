// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// ShakerOutput - the sound card behind the shaker sink. Owns a miniaudio
// playback device (ALSA on the Pi, WASAPI on Windows, the null backend
// for CI and for a machine with no card) and feeds it from ShakerDsp.
//
// Threads: the RT loop (or the haptics idle clock) calls push() once per
// control cycle, lock-free, gated on the open flag. miniaudio's audio
// thread calls the data callback, which renders from the rings, and its
// notification callback, which only flips atomics. open()/close()/
// reopen() are main/web thread, serialised by m_mx; the RT side never
// waits for them: close() drops the open flag first (pushes then skip
// the rings), then waits for a push already past it to finish (a push's
// length) before the device or the DSP is touched; open() raises the
// flag last. checkHealth() is the status thread's (through
// ShakerBank::info()).
//
// Failure is contained: a device that cannot be opened, or that goes
// away, leaves the sink offline (status says so, routes to shakers are
// silent) and the RT loop never knows. The card going away is seen two
// ways: the backend's own stopped/rerouted notification (WASAPI says;
// ALSA does not), and the card going quiet (no audio callback for
// kQuietSec while open: checkHealth). Either closes the gate and keeps
// the reason for error(); reopen() brings the card back when it is there.
// ============================================================

#include "ShakerDsp.h"
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace haptics {

struct ShakerDeviceInfo
{
    std::string name;
    bool        isDefault = false;
};

class ShakerOutput
{
public:
    ShakerOutput();
    ~ShakerOutput();

    // Playback devices the backend can see (main/web thread).
    std::vector<ShakerDeviceInfo> listDevices() const;

    // Open a device: deviceName "" = the system default, "null" = the
    // null backend (no hardware; CI), "null:N" = the null backend as a
    // pretend card with only N channels of its own (tests: a stereo
    // dongle asked for four shakers). channels 1..8, controlHz = the rate
    // push() will be called at. Returns false with the reason in error().
    bool open(const std::string& deviceName, int channels, double controlHz, int sampleRate = 48000);
    // Close and open again with the same request (web thread, once the
    // card is back after a loss). Returns open()'s answer.
    bool reopen();
    void close();
    bool isOpen() const { return m_open.load(std::memory_order_acquire); }

    // RT side: one sample per channel per control cycle, never blocks.
    // Channels beyond what the card itself has are pushed as silence, so
    // a stereo card asked for four never plays shaker 3 into shaker 1.
    void push(const double* samples, int n)
    {
        if (!m_open.load(std::memory_order_acquire)) return;
        // Counted in flight BEFORE the gate is looked at again: a closer
        // that shuts the gate and then waits for the count to reach 0
        // (closeLocked) cannot miss a push that got past it, so it never
        // reconfigures the DSP under one (the count and the second look
        // seq_cst, as the closer's store and wait). Never waits itself.
        m_pushesInFlight.fetch_add(1, std::memory_order_seq_cst);
        if (m_open.load(std::memory_order_seq_cst))
        {
            if (auto hook = m_pushHookForTest.load(std::memory_order_acquire)) hook(m_pushHookArg);
            const int own = m_channelsActual.load(std::memory_order_relaxed);
            m_dsp.push(samples, own > 0 ? std::min(n, own) : n);
            m_pushes.fetch_add(1, std::memory_order_relaxed);
        }
        m_pushesInFlight.fetch_sub(1, std::memory_order_release);
    }
    void setGain(int channel, double g) { m_dsp.setGain(channel, g); }

    // The card gone quiet: no audio callback for this long while open
    // means it is no longer pulling data (unplugged, or the backend's
    // thread wedged). A healthy card calls every few ms.
    static constexpr double kQuietSec = 2.0;
    // Status thread: compare the callback count with the last look; a
    // quiet card is marked lost, and a card marked quiet that is calling
    // back again (it only paused) is put back in play. Cheap, takes m_mx.
    void checkHealth();
    // The card went away (notification, or quiet): close the gate and keep
    // the reason; the device itself is left for reopen() or close(). Any
    // thread, atomics only.
    void markLost(const char* why);

    // Status (any thread).
    std::string deviceName() const { std::lock_guard<std::mutex> lk(m_mx); return m_deviceName; }
    std::string error() const;
    int      channels() const      { return m_dsp.channels(); }           // what is rendered (the channels asked for)
    int      channelsActual() const { return m_channelsActual.load(std::memory_order_relaxed); }   // what the card itself has
    int      sampleRate() const    { return m_sampleRate; }
    uint64_t underruns() const     { return m_dsp.underruns(); }
    uint64_t callbacks() const     { return m_callbacks.load(std::memory_order_relaxed); }
    uint64_t pushes() const        { return m_pushes.load(std::memory_order_relaxed); }
    int      fill() const          { return m_dsp.fill(0); }
    float    level(int channel) const { return m_dsp.level(channel); }
    // Latency the backend reports for its buffering, ms (0 when unknown).
    double   bufferMs() const      { return m_bufferMs; }
    // The whole delay from the control loop to the card's output, ms: the
    // ring's target plus the card's buffer.
    double   latencyMs() const     { return m_dsp.targetMs() + m_bufferMs; }
    // Windows: how many times the card's volume was found turned down
    // while open and put back to full (checkHealth).
    uint32_t volumeResets() const  { return m_volumeResets.load(std::memory_order_relaxed); }

    // Tests only: stop the card's audio callbacks without closing it (a
    // card gone quiet), and start them again. Held, the device's own stop
    // notification is ignored, as for our own close.
    void holdCallbacksForTest(bool hold);
    // Tests only: called inside push(), past the gate and before the DSP
    // (a push held there while the card is reopened). nullptr = none.
    void setPushHookForTest(void (*hook)(void*), void* arg)
    {
        m_pushHookArg = arg;
        m_pushHookForTest.store(hook, std::memory_order_release);
    }

private:
    struct Impl;
    Impl*  m_impl = nullptr;
    ShakerDsp m_dsp;
    std::atomic<bool> m_open{false};
    std::atomic<bool> m_closing{false};           // our own stop is not a loss
    std::atomic<int>  m_channelsActual{0};
    std::atomic<int>  m_lost{0};                  // 0 fine, else a kLost* reason
    std::atomic<uint64_t> m_callbacks{0}, m_pushes{0};
    std::atomic<int>  m_pushesInFlight{0};        // pushes past the gate and not yet out of the DSP
    std::atomic<void (*)(void*)> m_pushHookForTest{nullptr};
    void* m_pushHookArg = nullptr;
    mutable std::mutex m_mx;
    std::string m_deviceName, m_error;
    int    m_sampleRate = 48000;
    double m_bufferMs   = 0.0;
    // The request, for reopen().
    std::string m_reqDevice; int m_reqChannels = 2; double m_reqControlHz = 2000.0; int m_reqRate = 48000; bool m_reqValid = false;
    // Health: the callback count at the last look and when that was.
    uint64_t m_healthCallbacks = 0; int64_t m_healthMs = 0;
    // Windows: the card's volume, last checked when (ms) and put back how often.
    int64_t m_volumeCheckMs = 0;
    std::atomic<uint32_t> m_volumeResets{0};

    void closeLocked();
    static void dataCallback(void* device, void* out, const void* in, unsigned frames);
    static void notificationCallback(const void* notification);
};

} // namespace haptics
