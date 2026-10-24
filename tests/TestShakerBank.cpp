// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestShakerBank - two sound cards as one row of shakers, on miniaudio's
// null backend (no hardware; the device pulls audio on its own timer):
//   SB-1 {null,2} + {null,4}: six shakers, numbered across the devices,
//        a tone pushed on shaker 3 plays on device 2's first channel and
//        nowhere else; info() has the first shaker and channel count of
//        each device;
//   SB-2 a card with fewer channels than asked for ("null:2" asked for
//        four, the stereo dongle case): the status shows 2 of 4, shakers
//        3 and 4 are not live, and what is pushed on them is silence
//        (never folded into shakers 1 and 2);
//   SB-3 the card going away: markLost (what the backend's stopped and
//        rerouted notifications and the quiet watchdog call) closes the
//        gate, the status says why, pushes are skipped; reopen() from
//        the web thread brings it back and the tone plays again.
//   SB-4 the shutdown's bounded close (runWithin): a call back in time is
//        waited for; one that hangs is left at the limit.
//   SB-5 a push already past the gate when the card is reopened (a card
//        lost, Reopen on the dashboard): the reopen waits for it before it
//        resets the DSP, so the ring is never reset under a push.
//
//   ctest -R ShakerBank
// ============================================================
#include "ShakerBank.h"
#include "BoundedCall.h"

#include <atomic>
#include <chrono>
#include <memory>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

static int g_fail = 0, g_pass = 0;
static void check(bool ok, const char* name)
{
    if (ok) { ++g_pass; std::printf("[PASS] %s\n", name); }
    else    { ++g_fail; std::printf("[FAIL] %s\n", name); }
}

static const double kPi = 3.14159265358979;

// Push a 40 Hz tone of level A on one shaker (others silent) at the
// control rate, in real time, for `ms`: the null device consumes on its
// own clock, so the producer must keep pace with it.
template <class Sink>   // a ShakerBank, or one ShakerOutput
static void playTone(Sink& bank, int shaker, double A, int ms, double controlHz = 2000.0)
{
    using clock = std::chrono::steady_clock;
    const auto t0 = clock::now();
    long pushed = 0;
    while (true)
    {
        const double elapsed = std::chrono::duration<double>(clock::now() - t0).count();
        if (elapsed * 1000.0 >= ms) break;
        const long due = static_cast<long>(elapsed * controlHz);
        for (; pushed < due; ++pushed)
        {
            double s[haptics::MAX_SHAKER_OUT] = {};
            if (shaker >= 0) s[shaker] = A * std::sin(2 * kPi * 40.0 * pushed / controlHz);
            bank.push(s, haptics::MAX_SHAKER_OUT);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
}

int main()
{
    // The null device's thread at normal priority (as the test runs are).
#ifdef _WIN32
    _putenv_s("NULLCAT_NO_RT", "1");
#else
    setenv("NULLCAT_NO_RT", "1", 1);
#endif

    // ---- SB-1 ----
    {
        haptics::ShakerBank bank;
        bank.open({ { "null", 2 }, { "null", 4 } }, 2000.0);
        check(bank.deviceCount() == 2 && bank.totalChannels() == 6, "SB-1 two null devices, 2 + 4 channels, are six shakers");
        check(bank.anyOpen(), "SB-1 both opened");
        auto info = bank.info();
        check(info.size() == 2 && info[0].open && info[1].open, "SB-1 info() lists both, open");
        check(info[0].firstShaker == 0 && info[0].channels == 2 && info[0].channelsWanted == 2, "SB-1 device 1: shakers 1-2 (first 0, 2 channels)");
        check(info[1].firstShaker == 2 && info[1].channels == 4 && info[1].channelsWanted == 4, "SB-1 device 2: shakers 3-6 (first 2, 4 channels)");
        check(info[0].error.empty() && info[1].error.empty(), "SB-1 no error on either");
        for (int s = 0; s < 6; ++s) check(bank.channelLive(s), "SB-1 every shaker 1-6 is live");
        check(!bank.channelLive(6) && !bank.channelLive(-1), "SB-1 shaker 7 (and -1) is not");

        playTone(bank, 2, 0.5, 600);   // shaker 3 = device 2, channel 1
        info = bank.info();
        std::printf("  SB-1 levels: device 1 [%.3f %.3f], device 2 [%.3f %.3f %.3f %.3f]\n",
                    info[0].levels[0], info[0].levels[1], info[1].levels[0], info[1].levels[1], info[1].levels[2], info[1].levels[3]);
        check(info[1].levels.size() == 4 && info[1].levels[0] > 0.4 && info[1].levels[0] < 0.6,
              "SB-1 a 0.5 tone on shaker 3 reads on device 2's first channel");
        check(info[0].levels[0] < 0.02 && info[0].levels[1] < 0.02 && info[1].levels[1] < 0.02 && info[1].levels[2] < 0.02 && info[1].levels[3] < 0.02,
              "SB-1 ...and nowhere else");
        // (Underruns are not asserted: the device ran on its own clock
        // between open() and the tone, draining the primed ring, and a
        // drained ring refills at the servo's 0.5 % for seconds. The DSP
        // tests pin the servo; this suite pins the fan-out.)
        bank.close();
        check(!bank.anyOpen() && bank.deviceCount() == 0, "SB-1 close() empties the bank");
    }

    // ---- SB-2 ----
    {
        // A stereo card asked for four shakers. Before: the status showed
        // four live channels and miniaudio folded shakers 3 and 4 into 1
        // and 2. Reverting the channelsActual cap in ShakerOutput::push and
        // the channelsActual use in ShakerBank::info/channelLive fails
        // these.
        haptics::ShakerBank bank;
        bank.open({ { "null:2", 4 } }, 2000.0);
        auto info = bank.info();
        check(info.size() == 1 && info[0].open, "SB-2 a null:2 card asked for 4 opens");
        check(info[0].channels == 2 && info[0].channelsWanted == 4, "SB-2 the status shows the card's own 2 channels of the 4 wanted");
        check(bank.totalChannels() == 4, "SB-2 the numbering still reserves 4 (the next device's shakers do not move)");
        check(bank.channelLive(0) && bank.channelLive(1), "SB-2 shakers 1 and 2 are live");
        check(!bank.channelLive(2) && !bank.channelLive(3), "SB-2 shakers 3 and 4 are not (the card has no channel for them)");
        playTone(bank, 2, 0.5, 600);   // shaker 3: the card has no channel for it
        info = bank.info();
        std::printf("  SB-2 levels with a tone on shaker 3: [%.3f %.3f %.3f %.3f]\n", info[0].levels[0], info[0].levels[1], info[0].levels[2], info[0].levels[3]);
        check(info[0].levels.size() == 4 && info[0].levels[0] < 0.02 && info[0].levels[1] < 0.02,
              "SB-2 a tone on shaker 3 is NOT folded into shakers 1 or 2");
        check(info[0].levels[2] < 0.02 && info[0].levels[3] < 0.02, "SB-2 ...and the lost channels read silent");
        playTone(bank, 0, 0.5, 600);
        info = bank.info();
        check(info[0].levels[0] > 0.4 && info[0].levels[0] < 0.6 && info[0].levels[1] < 0.02, "SB-2 a tone on shaker 1 plays on the card's first channel");
        bank.close();
    }

    // ---- SB-2b ----
    {
        // The same at the card itself. The status above shows only the
        // card's own channels, so it read silent for shakers 3 and 4 even
        // with the cap gone; and the null backend never folds. What the
        // card is fed on its channel 3 is the push the cap must stop:
        // reverting the channelsActual cap in ShakerOutput::push fails this.
        haptics::ShakerOutput out;
        check(out.open("null:2", 4, 2000.0), "SB-2b a null:2 card asked for 4 opens on its own");
        playTone(out, 2, 0.5, 600);   // shaker 3: the card has no channel for it
        std::printf("  SB-2b the card's own levels with a tone on shaker 3: [%.3f %.3f %.3f %.3f]\n",
                    out.level(0), out.level(1), out.level(2), out.level(3));
        check(out.level(2) < 0.02 && out.level(0) < 0.02 && out.level(1) < 0.02,
              "SB-2b a tone pushed on shaker 3 reaches no channel of the card (pushed as silence)");
        out.close();
    }

    // ---- SB-2c the quiet watchdog ----
    {
        // A card that stops calling for audio without a word (a USB card
        // wedged or pulled where the backend never says) is marked lost
        // once it has been quiet for kQuietSec, and put back in play by
        // itself once it calls again. Nothing exercised it before.
        haptics::ShakerOutput out;
        check(out.open("null", 2, 2000.0), "SB-2c a null card opens");
        out.checkHealth();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        out.checkHealth();
        check(out.isOpen() && out.error().empty(), "SB-2c ...calling back: open, no error");
        out.holdCallbacksForTest(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        out.checkHealth();   // the last callback count, from here
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(haptics::ShakerOutput::kQuietSec * 1000.0) + 300));
        out.checkHealth();
        check(!out.isOpen() && out.error().find("gone quiet") != std::string::npos,
              "SB-2c quiet past kQuietSec: marked lost, the status says it has gone quiet");
        out.holdCallbacksForTest(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        out.checkHealth();
        check(out.isOpen() && out.error().empty(), "SB-2c ...calling again: back in play by itself, no error");
        out.close();
    }

    // ---- SB-3 ----
    {
        haptics::ShakerBank bank;
        bank.open({ { "null", 2 } }, 2000.0);
        auto info = bank.info();
        check(info[0].open && info[0].error.empty(), "SB-3 open, no error");
        playTone(bank, 0, 0.5, 300);
        // The card goes away. ShakerOutput::markLost is what the backend's
        // notification callback (stopped / rerouted) and the quiet
        // watchdog (checkHealth) call; the null backend never fires
        // either, so it is called here. Reverting markLost to a no-op
        // fails the next three.
        {
            haptics::ShakerOutput lone;
            check(lone.open("null", 2, 2000.0), "SB-3 a lone output opens");
            lone.markLost("stopped");
            check(!lone.isOpen(), "SB-3 a stopped notification closes the gate");
            check(lone.error().find("stopped") != std::string::npos, "SB-3 ...and the error says the card stopped");
            const uint64_t p0 = lone.pushes();
            double z[haptics::MAX_SHAKERS] = { 0.5 };
            for (int i = 0; i < 100; ++i) lone.push(z, 2);
            check(lone.pushes() == p0, "SB-3 pushes are skipped while lost (the RT side never touches a lost device's rings)");
            check(lone.reopen() && lone.isOpen() && lone.error().empty(), "SB-3 reopen() opens the same request again, error cleared");
            for (int i = 0; i < 100; ++i) lone.push(z, 2);
            check(lone.pushes() == p0 + 100, "SB-3 ...and pushes land again");
            lone.markLost("rerouted");
            check(!lone.isOpen() && lone.error().find("moved") != std::string::npos, "SB-3 a rerouted notification closes the gate with its own reason");
            lone.markLost("quiet");   // a second loss changes nothing
            check(lone.error().find("moved") != std::string::npos, "SB-3 ...and the first reason is kept");
            // The watchdog: a running null device keeps calling back, so
            // two health checks 2.2 s apart leave it open (no false alarm).
            check(lone.reopen(), "SB-3 reopened for the watchdog check");
            lone.checkHealth();
            std::this_thread::sleep_for(std::chrono::milliseconds(2200));
            lone.checkHealth();
            check(lone.isOpen(), "SB-3 the quiet watchdog does not fire on a card that is calling back");
            // A card marked quiet that calls back again only paused (the
            // machine slept): the next health check puts it back in play.
            lone.markLost("quiet");
            check(!lone.isOpen() && lone.error().find("quiet") != std::string::npos, "SB-3 a quiet card is marked lost with its reason");
            std::this_thread::sleep_for(std::chrono::milliseconds(60));
            lone.checkHealth();
            check(lone.isOpen() && lone.error().empty(), "SB-3 ...and comes back by itself once it is calling back again");
            lone.close();
            check(!lone.isOpen() && lone.error().empty(), "SB-3 our own close() is not a loss (no reason kept)");
        }
        // Through the bank: a device marked lost shows in info() and in
        // channelLive(); bank.reopen() brings every device back.
        bank.close();
        bank.open({ { "null", 2 }, { "null", 2 } }, 2000.0);
        check(bank.channelLive(2), "SB-3 bank: shaker 3 live on device 2");
        // (ShakerBank keeps its devices private; the loss is simulated on
        // a device of its own above. Here: reopen() on a healthy bank
        // touches no device, reports every one open, and leaves every
        // shaker live.)
        check(bank.reopen(), "SB-3 bank.reopen() on a healthy bank: every device open");
        info = bank.info();
        check(info.size() == 2 && info[0].open && info[1].open && info[0].firstShaker == 0 && info[1].firstShaker == 2,
              "SB-3 ...with the numbering unchanged");
        playTone(bank, 3, 0.5, 600);
        info = bank.info();
        check(info[1].levels[1] > 0.4 && info[1].levels[1] < 0.6 && info[0].levels[0] < 0.02,
              "SB-3 ...and a tone on shaker 4 plays on device 2's second channel after the reopen");
        bank.close();
    }

    // ---- SB-4 the shutdown's bounded close (runWithin) ----
    // A card unplugged while open hung a Pi's shutdown in its close until
    // systemd killed the service (the learned gear ratios lost). The
    // shutdown now gives the close a few seconds and goes on without it.
    // A call that comes back in time is waited for and ran; one that does
    // not is left behind at the limit, not waited for (joining it instead
    // fails the timing check), and still finishes on its own.
    {
        bool ran = false;
        check(runWithin([&ran]() { ran = true; }, std::chrono::milliseconds(500)) && ran,
              "SB-4 runWithin: a call that comes back in time is waited for (true, it ran)");
        auto finished = std::make_shared<std::atomic<bool>>(false);
        const auto t0 = std::chrono::steady_clock::now();
        const bool done = runWithin([finished]() { std::this_thread::sleep_for(std::chrono::milliseconds(600)); finished->store(true); },
                                    std::chrono::milliseconds(50));
        const double waitedMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        check(!done && waitedMs < 400.0, "SB-4 runWithin: a call that hangs is left at the limit (false, not waited for)");
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
        check(finished->load(), "SB-4 ...and the abandoned call still runs to its end on its own");
    }

    // ---- SB-5 a reopen waits for a push already past the gate ----
    // The RT side pushes every half millisecond; one push is held inside
    // (past the gate, before the DSP) for 150 ms while another thread
    // reopens the card. The reopen must not reset the DSP until that push
    // is out (it used to: the gate only stops pushes that have not begun).
    {
        struct Hold { std::atomic<int> state{0}; };   // 0 off, 1 armed, 2 held inside, 3 out
        Hold hold;
        haptics::ShakerOutput out;
        check(out.open("null", 2, 2000.0), "SB-5 a null card opens");
        out.setPushHookForTest([](void* a)
        {
            auto* h = static_cast<Hold*>(a);
            int armed = 1;
            if (h->state.compare_exchange_strong(armed, 2))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(150));
                h->state.store(3);
            }
        }, &hold);
        std::atomic<bool> run{true};
        std::thread rt([&]
        {
            const double z[2] = { 0.0, 0.0 };
            while (run.load()) { out.push(z, 2); std::this_thread::sleep_for(std::chrono::microseconds(500)); }
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        hold.state.store(1);
        while (hold.state.load() != 2) std::this_thread::yield();   // a push is held inside now
        const bool reopened = out.reopen();
        const bool outFirst = hold.state.load() == 3;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        run.store(false);
        rt.join();
        out.setPushHookForTest(nullptr, nullptr);
        check(reopened, "SB-5 the card reopens with a push held inside");
        check(outFirst, "SB-5 ...and the reopen waited for that push before it reset the DSP");
        check(out.fill() >= 0 && out.fill() <= haptics::SHAKER_RING, "SB-5 ...so the ring is whole after");
        out.close();
    }

    std::printf("\nTestShakerBank: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
