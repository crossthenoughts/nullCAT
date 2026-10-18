// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// runWithin - run a call on a thread of its own and wait for it no longer
// than the limit. A call that does not come back in time is left running
// (detached) and false is returned, so a shutdown can go on without it.
//
// For teardown only. The abandoned call may hold whatever it holds (a sound
// card's close waiting on a card unplugged while open: a Pi's shutdown hung
// there until systemd killed it, and the learned gear ratios were lost), so
// the caller then exits without running the destructors that would wait on
// it again (std::_Exit after its last log line).
// ============================================================

#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

inline bool runWithin(std::function<void()> call, std::chrono::milliseconds limit)
{
    // Shared with the thread: it outlives this function when the call is
    // abandoned.
    struct State { std::mutex mx; std::condition_variable cv; bool done = false; };
    auto st = std::make_shared<State>();
    std::thread t([st, call = std::move(call)]()
    {
        call();
        std::lock_guard<std::mutex> lk(st->mx);
        st->done = true;
        st->cv.notify_all();
    });
    bool done;
    {
        std::unique_lock<std::mutex> lk(st->mx);
        done = st->cv.wait_for(lk, limit, [&] { return st->done; });
    }
    if (done) t.join();
    else      t.detach();
    return done;
}
