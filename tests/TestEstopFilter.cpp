// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
//
// TestEstopFilter - the panel e-stop input's level filter, no GPIO:
//   F-1 one open poll is a blip: reported once, never engages (the Pi's
//       line read open for exactly one 10 ms poll with nobody at the button);
//   F-2 two consecutive open polls engage, on the second, once;
//   F-3 release takes releaseSamples consecutive closed polls; one open poll
//       inside the run starts it again;
//   F-4 a closed poll resets the engage count;
//   F-5 the panel's own filter: 2 polls to engage, 0.5 s at 10 ms to release;
//   F-6 counts below 1 are clamped to 1 (the first open poll engages).
#include "../src/EstopFilter.h"
#include <cstdio>

static int g_pass = 0, g_fail = 0;
static void check(bool ok, const char* what)
{
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    (ok ? g_pass : g_fail)++;
}

using Ev = EstopFilter::Event;

int main()
{
    // ---- F-1 a single open poll is a blip ----
    {
        EstopFilter f(2, 50);
        bool anyEngage = false;
        for (int i = 0; i < 100; ++i) anyEngage |= (f.step(false) != Ev::None);
        check(!anyEngage && !f.engaged(), "F-1 closed input: nothing happens");
        check(f.step(true) == Ev::None && !f.engaged(), "F-1 one open poll: not engaged");
        check(f.step(false) == Ev::Blip, "F-1 closed again: reported as a blip");
        check(f.lastBlipSamples() == 1, "F-1 the blip was one poll long");
        int blips = 0;
        for (int i = 0; i < 100; ++i) blips += (f.step(false) == Ev::Blip);
        check(blips == 0 && !f.engaged(), "F-1 the blip is reported once and never engages");
    }

    // ---- F-2 two consecutive open polls engage ----
    {
        EstopFilter f(2, 50);
        f.step(false);
        check(f.step(true) == Ev::None, "F-2 first open poll: waits");
        check(f.step(true) == Ev::Engaged && f.engaged(), "F-2 second open poll: engaged");
        int events = 0;
        for (int i = 0; i < 200; ++i) events += (f.step(true) != Ev::None);
        check(events == 0 && f.engaged(), "F-2 held open: engaged once, stays engaged");
    }

    // ---- F-3 release needs releaseSamples closed in a row ----
    {
        EstopFilter f(2, 50);
        f.step(true); f.step(true);
        int released = 0;
        for (int i = 0; i < 49; ++i) released += (f.step(false) == Ev::Released);
        check(released == 0 && f.engaged(), "F-3 49 closed polls: still engaged");
        check(f.step(true) == Ev::None && f.engaged(), "F-3 an open poll inside the run: still engaged");
        for (int i = 0; i < 49; ++i) released += (f.step(false) == Ev::Released);
        check(released == 0 && f.engaged(), "F-3 the run started again: 49 more closed, still engaged");
        check(f.step(false) == Ev::Released && !f.engaged(), "F-3 the 50th closed poll in a row: released");
        check(f.step(false) == Ev::None, "F-3 a release is not a blip");
    }

    // ---- F-4 a closed poll resets the engage count ----
    {
        EstopFilter f(3, 50);
        f.step(true); f.step(true);
        check(f.step(false) == Ev::Blip, "F-4 two open then closed (engage needs 3): a blip");
        check(f.lastBlipSamples() == 2, "F-4 the blip was two polls long");
        f.step(true); f.step(true);
        check(!f.engaged(), "F-4 two open polls after the reset: not engaged");
        check(f.step(true) == Ev::Engaged, "F-4 the third: engaged");
    }

    // ---- F-5 the panel's own filter (what pi/GpioPanel.cpp builds) ----
    {
        const int pollMs = 10;
        EstopFilter f = EstopFilter::forPanel(pollMs);
        check(f.step(true) == Ev::None, "F-5 panel: one open read waits");
        check(f.step(true) == Ev::Engaged, "F-5 panel: the second open read in a row engages (20 ms)");
        int polls = 0;
        while (f.engaged() && polls < 1000) { f.step(false); ++polls; }
        check(polls * pollMs == 500, "F-5 panel: released after 0.5 s closed at a 10 ms poll");
    }

    // ---- F-6 clamps ----
    {
        EstopFilter f(0, 0);
        check(f.step(true) == Ev::Engaged, "F-6 engage count 0 is clamped to 1");
        check(f.step(false) == Ev::Released, "F-6 release count 0 is clamped to 1");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
