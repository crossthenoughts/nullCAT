// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// EstopFilter.h - the panel e-stop input's level filter
//
// The GPIO e-stop input is polled (every 10 ms on the Pi). One poll reading
// open is not an e-stop: the line has been seen to read open for exactly one
// poll with nobody at the button. The filter
//   - engages once the input reads open on engageSamples consecutive polls;
//   - releases once it reads closed on releaseSamples consecutive polls;
//   - reports a shorter open run as a blip (to log, never to act on).
// The input is a software convenience. The mushroom's hard-wired stop chain
// to the drives is the safety device and nothing here filters it.
//
// Pure and header-only, so it is unit tested on every platform
// (TestEstopFilter).
// ============================================================

class EstopFilter
{
public:
    enum class Event { None, Engaged, Released, Blip };

    EstopFilter(int engageSamples, int releaseSamples)
        : m_engageN(engageSamples < 1 ? 1 : engageSamples),
          m_releaseN(releaseSamples < 1 ? 1 : releaseSamples) {}

    // One poll: open = the input reads open (stop).
    Event step(bool open)
    {
        if (open == m_engaged)
        {
            // Back to the settled level. An open run that ended before it
            // engaged was a blip.
            const bool blip = !m_engaged && m_run > 0;
            if (blip) m_lastBlipSamples = m_run;
            m_run = 0;
            return blip ? Event::Blip : Event::None;
        }
        if (++m_run >= (m_engaged ? m_releaseN : m_engageN))
        {
            m_engaged = open;
            m_run = 0;
            return m_engaged ? Event::Engaged : Event::Released;
        }
        return Event::None;
    }

    // The panel's filter (pi/GpioPanel.cpp) at its poll period: open on
    // two reads in a row to engage, closed for 0.5 s to release.
    static constexpr int    kPanelEngageReads = 2;
    static constexpr double kPanelReleaseSec  = 0.5;
    static EstopFilter forPanel(int pollMs)
    {
        const int ms = pollMs > 0 ? pollMs : 1;
        return EstopFilter(kPanelEngageReads, static_cast<int>(kPanelReleaseSec * 1000.0 / ms + 0.5));
    }

    bool engaged() const { return m_engaged; }
    // Length in polls of the last blip.
    int  lastBlipSamples() const { return m_lastBlipSamples; }

private:
    int  m_engageN;
    int  m_releaseN;
    bool m_engaged = false;
    int  m_run = 0;              // consecutive polls disagreeing with m_engaged
    int  m_lastBlipSamples = 0;
};
