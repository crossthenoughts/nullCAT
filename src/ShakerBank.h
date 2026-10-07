// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// ShakerBank - one or more sound cards as one row of shaker channels.
// Shaker numbers run across the devices in order: a stereo dongle as
// device 1 and another as device 2 give shakers 1..4. Each device keeps
// its own rings and drift servo (its own crystal); push() fans one
// control cycle's samples out to all of them, lock-free.
// ============================================================

#include "HapticsTypes.h"   // MAX_SHAKER_OUT
#include "ShakerOutput.h"
#include <memory>
#include <string>
#include <vector>

namespace haptics {

struct ShakerDeviceSpec
{
    std::string device;      // name substring, "" = default, "null" = no hardware
    int         channels = 2;
};

class ShakerBank
{
public:
    // Open every device in the list; a device that fails is kept in the
    // list (offline, its channels silent) so numbering stays stable.
    void open(const std::vector<ShakerDeviceSpec>& specs, double controlHz)
    {
        close();
        int base = 0;
        for (const ShakerDeviceSpec& sp : specs)
        {
            if (base >= MAX_SHAKER_OUT) break;
            Slot s;
            s.out = std::make_unique<ShakerOutput>();
            s.base = base;
            s.want = std::max(1, std::min(MAX_SHAKER_OUT - base, sp.channels));
            s.spec = sp;
            s.ok   = s.out->open(sp.device, s.want, controlHz);
            base  += s.want;
            m_slots.push_back(std::move(s));
        }
        m_total = base;
    }
    void close()
    {
        for (Slot& s : m_slots) if (s.out) s.out->close();
        m_slots.clear();
        m_total = 0;
    }

    // RT side.
    void push(const double* samples, int n)
    {
        for (Slot& s : m_slots)
            if (s.ok) s.out->push(samples + s.base, std::max(0, std::min(s.want, n - s.base)));
    }

    // Any device open?
    bool anyOpen() const { for (const Slot& s : m_slots) if (s.ok && s.out->isOpen()) return true; return false; }
    int  totalChannels() const { return m_total; }
    int  deviceCount() const   { return static_cast<int>(m_slots.size()); }

    // Per device, for status.
    struct Info
    {
        std::string wanted, name, error;
        bool   open = false;
        int    firstShaker = 0, channels = 0, sampleRate = 0;
        double bufferMs = 0.0;
        uint64_t underruns = 0;
        int    fill = 0;
        std::vector<float> levels;
    };
    std::vector<Info> info() const
    {
        std::vector<Info> out;
        for (const Slot& s : m_slots)
        {
            Info i;
            i.wanted = s.spec.device; i.firstShaker = s.base; i.channels = s.want;
            if (s.out)
            {
                i.open = s.ok && s.out->isOpen();
                i.name = s.out->deviceName(); i.error = s.out->error();
                i.sampleRate = s.out->sampleRate(); i.bufferMs = s.out->bufferMs();
                i.underruns = s.out->underruns(); i.fill = s.out->fill();
                for (int c = 0; c < s.want; ++c) i.levels.push_back(i.open ? s.out->level(c) : 0.0f);
            }
            out.push_back(i);
        }
        return out;
    }

    // The devices the backend can see (the first slot's context, or a
    // scratch one when nothing is open).
    std::vector<ShakerDeviceInfo> listDevices() const
    {
        if (!m_slots.empty() && m_slots.front().out) return m_slots.front().out->listDevices();
        ShakerOutput scratch;
        return scratch.listDevices();
    }

    // Is this shaker number backed by an open device?
    bool channelLive(int shaker) const
    {
        for (const Slot& s : m_slots)
            if (shaker >= s.base && shaker < s.base + s.want) return s.ok && s.out->isOpen();
        return false;
    }

private:
    struct Slot
    {
        std::unique_ptr<ShakerOutput> out;
        ShakerDeviceSpec spec;
        int  base = 0, want = 0;
        bool ok = false;
    };
    std::vector<Slot> m_slots;
    int m_total = 0;
};

} // namespace haptics
