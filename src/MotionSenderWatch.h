// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// MotionSenderWatch: two programs sending the motion line at once.
//
// The motion line has no arbitration: the last line to arrive is the one
// the rig follows. Two motion programs left on together (one of them meant
// to send the haptic channels only) make the rig jump between two cues.
// This only WATCHES and says so; nothing is set aside (a lock would cut off
// a sender whose source port changes, and what the motion programs do
// there is not known).
//
// A sender is its address and port (TelemetryInput's sender key). Two are
// sending when, over kWindowMs, each sent at least kMinLines, from within
// kLiveMs of the window's start to within kLiveMs of its end, for kWindows
// windows in a row: both there for the whole of two seconds, so a second
// program is said two to three seconds after it starts. Not two senders:
// a program restarted on another port, or one handed over to another
// (under 1.5 s of both: no two whole windows), a sender whose port changes
// (no key spans a window), a stray sender with a few lines a second, a
// third sender while two are live (not counted).
//
// Pure, receive path only (one thread), no allocation.
// ============================================================
#include <cstdint>

struct MotionSenderWatch
{
    static constexpr int64_t kWindowMs = 1000;   // judged over this long
    static constexpr int     kMinLines = 5;      // each sender's lines in it, at least
    static constexpr int64_t kLiveMs   = 250;    // ...its first within this of the window's start, its last of its end
    static constexpr int     kWindows  = 2;      // windows in a row before it counts

    struct Src { uint64_t key = 0; int64_t firstMs = 0, lastMs = 0; int n = 0; };   // first and last line this window
    Src     a, b;
    int64_t windowStartMs = 0;
    int     run   = 0;       // windows in a row that found two
    bool    mixed = false;   // two programs are sending (as of the last window judged)

    // A motion line from `sender` (0 = not known: never counted) at nowMs.
    // Returns mixed.
    bool note(uint64_t sender, int64_t nowMs)
    {
        if (sender == 0) return mixed;
        if (windowStartMs == 0) windowStartMs = nowMs;
        if (nowMs - windowStartMs >= kWindowMs)
        {
            const auto whole = [&](const Src& s) {
                return s.n >= kMinLines && s.firstMs - windowStartMs <= kLiveMs && nowMs - s.lastMs <= kLiveMs; };
            const bool two = whole(a) && whole(b);
            run   = two ? run + 1 : 0;
            mixed = run >= kWindows;
            a.n = b.n = 0;
            windowStartMs = nowMs;
        }
        Src* s = (a.key == sender) ? &a : (b.key == sender) ? &b : nullptr;
        if (!s)
        {
            // A new sender takes a slot that is free or has been quiet for a
            // window; with both slots live it is not counted.
            if (a.key == 0 || nowMs - a.lastMs >= kWindowMs)      s = &a;
            else if (b.key == 0 || nowMs - b.lastMs >= kWindowMs) s = &b;
            else return mixed;
            *s = Src{ sender, nowMs, nowMs, 0 };
        }
        if (s->n == 0) s->firstMs = nowMs;
        s->lastMs = nowMs;
        ++s->n;
        return mixed;
    }
};
