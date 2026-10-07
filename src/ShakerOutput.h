// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// ShakerOutput - the sound card behind the shaker sink. Owns a miniaudio
// playback device (ALSA on the Pi, WASAPI on Windows, the null backend
// for CI and for a machine with no card) and feeds it from ShakerDsp.
//
// Threads: the RT loop (or the haptics idle clock) calls push() once per
// control cycle, lock-free. miniaudio's audio thread calls the data
// callback, which renders from the rings. open()/close() are main/web
// thread and never run while the RT loop holds a reference: the owner
// stops pushing first (the enabled flag) and the rings are simply
// skipped when no device is open.
//
// Failure is contained: a device that cannot be opened, or that goes
// away, leaves the sink offline (status says so, routes to shakers are
// silent) and the RT loop never knows.
// ============================================================

#include "ShakerDsp.h"
#include <atomic>
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
    // null backend (no hardware; CI). channels 1..8, controlHz = the rate
    // push() will be called at. Returns false with the reason in error().
    bool open(const std::string& deviceName, int channels, double controlHz, int sampleRate = 48000);
    void close();
    bool isOpen() const { return m_open.load(std::memory_order_acquire); }

    // RT side: one sample per channel per control cycle, never blocks.
    void push(const double* samples, int n)
    {
        if (!m_open.load(std::memory_order_acquire)) return;
        m_dsp.push(samples, n);
        m_pushes.fetch_add(1, std::memory_order_relaxed);
    }
    void setGain(int channel, double g) { m_dsp.setGain(channel, g); }
    void setControlHz(double hz);

    // Status (any thread).
    std::string deviceName() const { std::lock_guard<std::mutex> lk(m_mx); return m_deviceName; }
    std::string error() const      { std::lock_guard<std::mutex> lk(m_mx); return m_error; }
    int      channels() const      { return m_dsp.channels(); }
    int      sampleRate() const    { return m_sampleRate; }
    uint64_t underruns() const     { return m_dsp.underruns(); }
    uint64_t callbacks() const     { return m_callbacks.load(std::memory_order_relaxed); }
    uint64_t pushes() const        { return m_pushes.load(std::memory_order_relaxed); }
    int      fill() const          { return m_dsp.fill(0); }
    float    level(int channel) const { return m_dsp.lastOut(channel); }
    // Latency the backend reports for its buffering, ms (0 when unknown).
    double   bufferMs() const      { return m_bufferMs; }

private:
    struct Impl;
    Impl*  m_impl = nullptr;
    ShakerDsp m_dsp;
    std::atomic<bool> m_open{false};
    std::atomic<uint64_t> m_callbacks{0}, m_pushes{0};
    mutable std::mutex m_mx;
    std::string m_deviceName, m_error;
    int    m_sampleRate = 48000;
    double m_bufferMs   = 0.0;

    static void dataCallback(void* device, void* out, const void* in, unsigned frames);
};

} // namespace haptics
