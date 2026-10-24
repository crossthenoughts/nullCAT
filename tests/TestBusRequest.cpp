// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestBusRequest - a software stop asked from another thread reaches the
// drives only through the bus owner (BusRequest.h, EtherCATMaster.h).
//
// Four real A6Drive objects write into a fake process image, as
// EtherCATMaster wires them. The "owner" below does what the RT loop's
// sendReceive(), the pump and the OP bridge do before each frame: take the
// request and, when there is one, disable every drive.
//
//   BR-1  a stop asked from another thread writes no controlword itself
//   BR-2  the owner's next frame disables every drive, once: the frame
//         after writes nothing (a drive enabled again stays enabled)
//   BR-3  several stops asked before a frame are one
//   BR-4  a stop asked between two owners (the loop stopped, the pump not
//         yet started) is carried out by the next owner's first frame
//   BR-5  many requesters, two owners taking turns through a join-style
//         handoff: no stop is lost (the last one asked is carried out),
//         and only the owner of the moment ever writes the drives
//
// Breaks with take() answering true twice for one request (BR-2), or a
// requester writing the drives itself (BR-1, BR-5). CI also runs it under
// ThreadSanitizer (ci.yml, tsan).
// ============================================================

#include "A6Drive.h"
#include "BusRequest.h"
#include "Logging.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <vector>

namespace {

int g_fail = 0;
void check(bool ok, const char* name)
{
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_fail;
}

constexpr uint16_t kEnabled  = 0x000F;   // enable operation
constexpr uint16_t kDisabled = 0x0003;   // what A6Drive::disableOperation writes: enable voltage + switch on, the quick-stop bit low (a DS402 quick stop)
constexpr int      kDrives   = 4;

// One drive's slice of the process image (CSP position layout).
struct Slot
{
    uint16_t cw = kEnabled; int8_t mode = 8; int32_t target = 0;
    uint16_t sw = 0x0237;   int8_t modeShown = 8; int32_t actual = 0;
};

struct Rig
{
    Slot     slots[kDrives];
    A6Drive  drives[kDrives];
    Rig()
    {
        for (int i = 0; i < kDrives; ++i)
        {
            drives[i].setSlaveIndex(i + 1);
            drives[i].setPDOPointers(&slots[i].cw, &slots[i].mode, &slots[i].target,
                                     &slots[i].sw, &slots[i].modeShown, &slots[i].actual);
        }
    }
    void enableAll() { for (Slot& s : slots) s.cw = kEnabled; }
    bool all(uint16_t cw) const { for (const Slot& s : slots) if (s.cw != cw) return false; return true; }
    // What the bus owner does before each frame (EtherCATMaster::serviceRequests).
    bool ownerFrame(BusRequest& stop)
    {
        if (!stop.take()) return false;
        for (A6Drive& d : drives) d.disableOperation();
        return true;
    }
};

} // namespace

int main()
{
    Logger::instance().setMinLevel(LogLevel::LVL_ERROR);

    {
        Rig rig; BusRequest stop;
        std::thread requester([&] { stop.request(); });
        requester.join();
        check(rig.all(kEnabled) && stop.pending(), "BR-1 a stop asked from another thread writes no controlword itself");
        check(rig.ownerFrame(stop) && rig.all(kDisabled), "BR-2 the owner's next frame disables every drive");
        rig.enableAll();
        check(!rig.ownerFrame(stop) && rig.all(kEnabled), "BR-2 ...once: the frame after writes nothing");
    }
    {
        Rig rig; BusRequest stop;
        for (int i = 0; i < 5; ++i) stop.request();
        const bool first = rig.ownerFrame(stop);
        rig.enableAll();
        const bool second = rig.ownerFrame(stop);
        check(first && !second && rig.all(kEnabled), "BR-3 several stops asked before a frame are one");
    }
    {
        // Owner A (the loop) runs frames, stops; the stop is asked with no
        // owner; owner B (the pump) starts and carries it out at once.
        Rig rig; BusRequest stop;
        std::atomic<bool> runA{true};
        std::thread a([&] { while (runA.load()) { rig.ownerFrame(stop); std::this_thread::yield(); } });
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        runA.store(false); a.join();                 // A has given up the bus
        stop.request();                              // asked between owners
        check(rig.all(kEnabled), "BR-4 a stop asked between two owners waits (nobody writes the drives)");
        bool firstFrame = false;
        std::thread b([&] { firstFrame = rig.ownerFrame(stop); });
        b.join();
        check(firstFrame && rig.all(kDisabled), "BR-4 ...and the next owner's first frame carries it out");
    }
    {
        // Two owners take turns (each joins before the other starts, as
        // stopPump() before the loop claims the bus); four requesters ask at
        // random. Each owner frame that carries out a stop records it and
        // then re-enables (as the loop's enable state machine would), so a
        // disabled drive at the end can only be a stop carried out by the
        // last owner after the last request. Only owners write the image.
        Rig rig; BusRequest stop;
        std::atomic<int> inFrame{0}, overlap{0};
        std::atomic<bool> requesting{true};
        std::atomic<long> asked{0}, done{0};
        std::vector<std::thread> requesters;
        for (int r = 0; r < 4; ++r)
            requesters.emplace_back([&, r]
            {
                unsigned x = 12345u + r;
                while (requesting.load())
                {
                    x = x * 1103515245u + 12345u;
                    if ((x >> 16) % 7 == 0) { stop.request(); asked.fetch_add(1); }
                    std::this_thread::yield();
                }
            });
        for (int turn = 0; turn < 400; ++turn)
        {
            std::thread owner([&]
            {
                for (int f = 0; f < 50; ++f)
                {
                    if (inFrame.fetch_add(1) != 0) overlap.fetch_add(1);
                    if (rig.ownerFrame(stop)) { done.fetch_add(1); rig.enableAll(); }
                    inFrame.fetch_sub(1);
                }
            });
            owner.join();
        }
        requesting.store(false);
        for (auto& t : requesters) t.join();
        // The last requests may have come after the last owner frame: one
        // more owner, then nothing is left.
        stop.request();
        std::thread last([&] { if (rig.ownerFrame(stop)) done.fetch_add(1); });
        last.join();
        std::printf("  BR-5: %ld stops asked, %ld carried out (several asked between two frames are one)\n",
                    asked.load(), done.load());
        check(overlap.load() == 0, "BR-5 two owners never write in the same moment");
        check(!stop.pending() && rig.all(kDisabled), "BR-5 no stop is lost: the last one asked is carried out");
        check(done.load() >= 1 && done.load() <= asked.load() + 1, "BR-5 a stop is carried out at most once per request");
    }

    std::printf("TestBusRequest: %s\n", g_fail ? "FAILED" : "all passed");
    return g_fail ? 1 : 0;
}
