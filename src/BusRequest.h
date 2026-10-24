// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// BusRequest - something any thread may ask of the drives (a software
// stop, a fault reset) that only the bus owner carries out.
//
// The bus owner is the one thread that writes the drives' process data
// (controlwords, targets, the fault-reset sequence) at any moment: the
// RT control loop while it runs, the background pump and the OP bridge
// before it while it does not, shutdown at the end (EtherCATMaster.h).
// A requester never writes them itself: it calls request(); the owner
// calls take() once per cycle (or frame) before it sends and carries the
// request out when it returns true.
//
// take() is true at most once per request (several requests before a
// take are one), so a request made while no thread owns the bus (the
// moment of a handoff, the loop stopping and the pump starting) is
// carried out by the next owner, and by one owner only.
//
// The independent hardware e-stop stays the safety layer (SAFETY.md);
// this is the software stop's path to the drives.
// ============================================================

#include <atomic>

class BusRequest
{
public:
    // Any thread.
    void request() { m_pending.store(true, std::memory_order_release); }
    // The bus owner only, once per cycle: true if a request was pending
    // (and it is now taken). The plain load first keeps the common case
    // (nothing asked) free of a read-modify-write every cycle.
    bool take()
    {
        return m_pending.load(std::memory_order_acquire)
            && m_pending.exchange(false, std::memory_order_acq_rel);
    }
    // Drop a request nobody needs carried out any more (shutdown, which
    // disables every drive itself).
    void clear() { m_pending.store(false, std::memory_order_release); }
    bool pending() const { return m_pending.load(std::memory_order_acquire); }

private:
    std::atomic<bool> m_pending{false};
};
