// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestCommandQueue - MotionController::enqueueCommand from several threads
// at once, as the web server's pool, the GPIO panel and the Qt window call
// it, with one consumer playing the RT thread's part (its own pop, through
// popCommandForTest). The queue underneath takes one producer: the
// controller must serialise them.
//
// Every command carries its producer and sequence number. Pass: every
// command enqueueCommand accepted arrives exactly once, none it refused
// arrives at all, and each arrives whole (its type is its producer's).
// The ring holds 31, so refusals are expected and fine.
//
// Breaks with the lock in enqueueCommand removed: two pushes at once write
// one slot twice (a lost or mixed command), or a late producer stores a
// stale write index and the consumer replays old slots (duplicates).
// CI also runs it under ThreadSanitizer (ci.yml, tsan).
//
// Usage: TestCommandQueue [commands per producer]   (default 200000)
// ============================================================

#include "MotionController.h"
#include "Logging.h"

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <thread>
#include <vector>

int main(int argc, char** argv)
{
    Logger::instance().setMinLevel(LogLevel::LVL_ERROR);
    const int per = (argc > 1 && std::atoi(argv[1]) > 0) ? std::atoi(argv[1]) : 200000;
    constexpr int kProducers = 6;
    // The queued command types (the haptics ones never reach the queue).
    static const MotionCommand::Type kTypes[] = {
        MotionCommand::Type::StartHoming, MotionCommand::Type::StartPark, MotionCommand::Type::StartUnpark,
        MotionCommand::Type::SlackBelts, MotionCommand::Type::TensionBelts,
        MotionCommand::Type::EngageDevice, MotionCommand::Type::ReleaseDevice };
    constexpr int kTypeCount = static_cast<int>(sizeof(kTypes) / sizeof(kTypes[0]));

    auto mc = std::make_unique<MotionController>();
    // accepted[p][s]: written only by producer p. seen[p][s]: only by the consumer.
    std::vector<std::vector<unsigned char>> accepted(kProducers, std::vector<unsigned char>(per, 0));
    std::vector<std::vector<unsigned int>>  seen(kProducers, std::vector<unsigned int>(per, 0));
    std::atomic<int>  ready{0};
    std::atomic<bool> go{false}, producersDone{false};
    long long outOfRange = 0, wrongType = 0;

    std::thread consumer([&]
    {
        MotionCommand c;
        auto take = [&]
        {
            while (mc->popCommandForTest(c))
            {
                const int p = c.intVal / per, s = c.intVal % per;
                if (c.intVal < 0 || p >= kProducers) { ++outOfRange; continue; }
                if (c.type != kTypes[p % kTypeCount]) ++wrongType;
                ++seen[p][s];
            }
        };
        ready.fetch_add(1);
        while (!go.load()) std::this_thread::yield();
        while (!producersDone.load()) take();
        take();   // what the producers left
    });
    std::vector<std::thread> producers;
    for (int p = 0; p < kProducers; ++p)
        producers.emplace_back([&, p]
        {
            ready.fetch_add(1);
            while (!go.load()) std::this_thread::yield();
            for (int s = 0; s < per; ++s)
            {
                MotionCommand c;
                c.type = kTypes[p % kTypeCount];
                c.intVal = p * per + s;
                if (mc->enqueueCommand(c)) accepted[p][s] = 1;
            }
        });
    while (ready.load() < kProducers + 1) std::this_thread::yield();
    go.store(true);
    for (auto& t : producers) t.join();
    producersDone.store(true);
    consumer.join();

    long long nAccepted = 0, nRefused = 0, lost = 0, duplicated = 0, phantom = 0;
    for (int p = 0; p < kProducers; ++p)
        for (int s = 0; s < per; ++s)
        {
            if (accepted[p][s]) ++nAccepted; else ++nRefused;
            if (accepted[p][s] && seen[p][s] == 0) ++lost;
            if (seen[p][s] > 1) duplicated += seen[p][s] - 1;
            if (!accepted[p][s] && seen[p][s] > 0) ++phantom;
        }
    std::printf("TestCommandQueue: %d producers x %d: %lld accepted, %lld refused (queue full); "
                "lost %lld, duplicated %lld, refused-but-delivered %lld, corrupt %lld\n",
                kProducers, per, nAccepted, nRefused, lost, duplicated, phantom, outOfRange + wrongType);

    int fail = 0;
    const auto check = [&](bool ok, const char* name)
    { std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name); if (!ok) ++fail; };
    check(nAccepted > 0, "commands get through under contention");
    check(lost == 0, "every accepted command is delivered");
    check(duplicated == 0, "no command is delivered twice (no stale slots replayed)");
    check(phantom == 0, "no refused command is delivered");
    check(outOfRange + wrongType == 0, "every command arrives whole");
    check(!mc->popCommandForTest(*std::make_unique<MotionCommand>()), "the queue is empty at the end");
    std::printf("TestCommandQueue: %s\n", fail ? "FAILED" : "all passed");
    return fail ? 1 : 0;
}
