// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// GpioPanel.cpp - physical control panel via libgpiod v2 (chardev API)
//
// Inputs use the internal pull-ups, so:
//   - Buttons (ENGAGE/PARK): momentary to GND → pressed reads INACTIVE (low).
//   - E-STOP (mushroom NC to GND): closed=OK reads INACTIVE (low);
//     open (pressed OR cut/unplugged cable) reads ACTIVE (high) → fail-safe.
// Outputs drive each LED HIGH through a series resistor (sourced by the pin).
//
// The poll runs ~100 Hz on a normal-priority thread; with isolcpus=2,3 it
// lands on cores 0/1 and never competes with the RT control loop.
// ============================================================

#include "GpioPanel.h"
#include "Logging.h"

#include <gpiod.h>
#include <chrono>
#include <vector>

namespace {
constexpr int    POLL_MS          = 10;   // ~100 Hz
constexpr int    DEBOUNCE_CYCLES  = 3;    // 3 × 10 ms = 30 ms stable
constexpr int    BLINK_HALF_CYCLES = 25;  // 25 × 10 ms = 250 ms → ~2 Hz blink

inline gpiod_line_value boolToVal(bool on)
{
    return on ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE;
}
} // namespace

GpioPanel::~GpioPanel()
{
    stop();
}

bool GpioPanel::start(const PanelPins& pins,
                      std::function<PanelStatus()> getStatus,
                      const PanelActions& actions)
{
    m_pins      = pins;
    m_getStatus = std::move(getStatus);
    m_actions   = actions;

    std::string path = pins.chip;
    if (path.rfind("/dev/", 0) != 0)
        path = "/dev/" + path;

    gpiod_chip* chip = gpiod_chip_open(path.c_str());
    if (!chip)
    {
        LOG_ERROR(strf("GpioPanel: cannot open %s (is the path right?). Panel disabled.",
                       path.c_str()));
        return false;
    }

    // Two settings groups: pulled-up inputs and (initially-off) outputs.
    gpiod_line_settings* in  = gpiod_line_settings_new();
    gpiod_line_settings* out = gpiod_line_settings_new();
    gpiod_line_config*   lc  = gpiod_line_config_new();
    gpiod_request_config* rc = gpiod_request_config_new();
    gpiod_line_request*  req = nullptr;

    if (in && out && lc && rc)
    {
        gpiod_line_settings_set_direction(in,  GPIOD_LINE_DIRECTION_INPUT);
        gpiod_line_settings_set_bias(in,       GPIOD_LINE_BIAS_PULL_UP);

        gpiod_line_settings_set_direction(out, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(out, GPIOD_LINE_VALUE_INACTIVE);

        // Request only the lines the active mode uses, so unused pins stay free.
        // E-STOP is always present; ENGAGE/PARK only with useButtons; LEDs only
        // with useLeds.
        std::vector<unsigned int> inOff;  inOff.push_back(pins.estop);
        if (pins.useButtons) { inOff.push_back(pins.engage); inOff.push_back(pins.park); }
        std::vector<unsigned int> outOff;
        if (pins.useLeds) { outOff = { pins.ledRun, pins.ledReady, pins.ledFault }; }

        bool ok = gpiod_line_config_add_line_settings(lc, inOff.data(), inOff.size(), in) == 0;
        if (ok && !outOff.empty())
            ok = gpiod_line_config_add_line_settings(lc, outOff.data(), outOff.size(), out) == 0;

        gpiod_request_config_set_consumer(rc, "nullcat-pi");

        if (ok)
            req = gpiod_chip_request_lines(chip, rc, lc);
    }

    // The request keeps its own copies; free the builders regardless.
    if (rc)  gpiod_request_config_free(rc);
    if (lc)  gpiod_line_config_free(lc);
    if (in)  gpiod_line_settings_free(in);
    if (out) gpiod_line_settings_free(out);

    if (!req)
    {
        LOG_ERROR("GpioPanel: could not request GPIO lines. Check the pin numbers "
                  "are free and the service user is in the 'gpio' group. Panel disabled.");
        gpiod_chip_close(chip);
        return false;
    }

    m_chip    = chip;
    m_request = req;
    m_running.store(true);
    m_thread  = std::thread(&GpioPanel::run, this);

    LOG_INFO(strf("GpioPanel: started on %s - estop=%u%s%s",
                  path.c_str(), pins.estop,
                  pins.useButtons ? strf(" engage=%u park=%u", pins.engage, pins.park).c_str() : " (no buttons)",
                  pins.useLeds ? strf(" LEDs run=%u ready=%u fault=%u", pins.ledRun, pins.ledReady, pins.ledFault).c_str() : " (no LEDs)"));
    return true;
}

void GpioPanel::run()
{
    auto* req = static_cast<gpiod_line_request*>(m_request);

    // Debounce state: a press fires once the raw level has been stable for
    // DEBOUNCE_CYCLES. Returns true only on a confirmed press edge.
    bool engageStable = false, parkStable = false;
    bool engageAutoStart = false;   // cold ENGAGE chains init -> start
    bool prevInitBusy    = false;
    int  engageCnt = 0, parkCnt = 0;
    auto edge = [](bool raw, bool& stable, int& cnt) -> bool
    {
        if (raw == stable) { cnt = 0; return false; }
        if (++cnt >= DEBOUNCE_CYCLES) { stable = raw; cnt = 0; return raw; }
        return false;
    };

    bool estopLatched = false;   // panel latch; no auto-resume on release
    int  tick = 0;
    int  pE = -1, pEn = -1, pP = -1;   // previous raw input states for change-logging

    while (m_running.load())
    {
        ++tick;

        // Inputs (pull-up): estop ACTIVE(high)=open=engaged; buttons INACTIVE(low)=pressed.
        // Button lines are read only when the mode uses them (short-circuit).
        const bool estopOpen  = gpiod_line_request_get_value(req, m_pins.estop) == GPIOD_LINE_VALUE_ACTIVE;
        const bool engageDown = m_pins.useButtons &&
            gpiod_line_request_get_value(req, m_pins.engage) == GPIOD_LINE_VALUE_INACTIVE;
        const bool parkDown   = m_pins.useButtons &&
            gpiod_line_request_get_value(req, m_pins.park)   == GPIOD_LINE_VALUE_INACTIVE;

        // Diagnostic: log raw inputs whenever any one changes, so wiring/polarity
        // can be verified live from journalctl. First pass prints the initial state.
        if ((int)estopOpen != pE || (int)engageDown != pEn || (int)parkDown != pP)
        {
            if (m_pins.useButtons)
                LOG_INFO(strf("GpioPanel: inputs  estop=%s  engage=%s  park=%s",
                              estopOpen ? "OPEN(stop)" : "closed(ok)",
                              engageDown ? "PRESSED" : "up", parkDown ? "PRESSED" : "up"));
            else
                LOG_INFO(strf("GpioPanel: inputs  estop=%s", estopOpen ? "OPEN(stop)" : "closed(ok)"));
            pE = estopOpen; pEn = engageDown; pP = parkDown;
        }

        const PanelStatus st = m_getStatus ? m_getStatus() : PanelStatus{};

        // E-STOP (level). Open (or cable cut) => assert + latch. Closed again =>
        // auto-release: the mushroom is the deliberate control, so releasing it
        // clears the e-stop and the UI (same effect as the web Release button).
        if (estopOpen)
        {
            if (!estopLatched)
            {
                estopLatched = true;
                if (m_actions.estop) m_actions.estop();
                LOG_WARNING("GpioPanel: E-STOP engaged (mushroom open / cable cut).");
            }
            else if (!st.estop && m_actions.estop)
            {
                m_actions.estop();   // re-assert if cleared elsewhere while still open
            }
        }
        else if (estopLatched)
        {
            estopLatched = false;
            if (m_actions.estopRelease) m_actions.estopRelease();
            LOG_INFO("GpioPanel: E-STOP released (mushroom closed).");
        }

        const bool engagePress = edge(engageDown, engageStable, engageCnt);
        const bool parkPress   = edge(parkDown,   parkStable,   parkCnt);

        if (engagePress)
        {
            if (estopLatched)
            {
                // Latch only persists while the mushroom is open; release it to clear.
                LOG_INFO("GpioPanel: ENGAGE ignored - release the E-STOP mushroom first.");
            }
            else if (st.loopRunning)
            {
                // Two-way button: from running, ENGAGE stops the loop -- and
                // ONLY the loop. It never de-inits: after a stop there is no
                // way to know whether the next wish is restart or shutdown,
                // so dropping the bus stays a deliberate act (web UI).
                if (m_actions.stop) m_actions.stop();
                engageAutoStart = false;
                LOG_INFO("GpioPanel: ENGAGE → Stop loop (bus stays up; de-init via web UI).");
            }
            else if (!st.masterOp && !st.initBusy)
            {
                if (m_actions.init) m_actions.init();
                engageAutoStart = true;   // cold press means "run": chain the start
                LOG_INFO("GpioPanel: ENGAGE → Initialize EtherCAT (loop will auto-start).");
            }
            else if (st.masterOp && !st.loopRunning)
            {
                if (m_actions.start) m_actions.start();
                LOG_INFO("GpioPanel: ENGAGE → Start loop.");
            }
        }

        // Cold-press chain: a single ENGAGE from offline means "make it run",
        // so start the loop as soon as the init it triggered reaches OP. The
        // intent is dropped the moment anything interrupts: e-stop, a failed
        // init (initBusy cleared without OP), or ENGAGE-stop above.
        if (engageAutoStart)
        {
            if (estopLatched || st.fault)
                engageAutoStart = false;
            else if (st.masterOp && !st.loopRunning && !st.initBusy)
            {
                engageAutoStart = false;
                if (m_actions.start) m_actions.start();
                LOG_INFO("GpioPanel: ENGAGE chain → init complete, starting loop.");
            }
            else if (!st.masterOp && !st.initBusy && prevInitBusy)
            {
                engageAutoStart = false;
                LOG_INFO("GpioPanel: ENGAGE chain cancelled - init did not reach OP.");
            }
        }
        prevInitBusy = st.initBusy;

        if (parkPress && !estopLatched && st.loopRunning)
        {
            if (st.parked)
            {
                if (m_actions.unpark) m_actions.unpark();
                LOG_INFO("GpioPanel: UNPARK.");
            }
            else
            {
                if (m_actions.park) m_actions.park();
                LOG_INFO("GpioPanel: PARK.");
            }
        }

        // LEDs (only if the mode has them). estop wins, then fault (blink),
        // then running/ready. A requested self-test pre-empts the normal state.
        if (m_pins.useLeds)
        {
            if (m_ledTestReq.exchange(false))
                runLedTest(req);

            // Timings: fast ~2 Hz blink; slow toggles every 1 s (2 s period).
            const bool blink    = ((tick / BLINK_HALF_CYCLES) % 2) != 0;
            const bool slow1s   = ((tick / 100) % 2) != 0;
            const bool estopNow = st.estop || estopLatched;

            bool red = false, amber = false, green = false;
            if (estopNow)            red = true;                    // solid
            else if (st.fault)       red = blink;                  // blink
            else if (st.initBusy)    amber = blink;                // initializing
            else if (st.masterOp && st.homing) { green = slow1s; amber = !slow1s; }  // alternate, offset 1 s
            else if (st.masterOp && st.parked)   green = slow1s;   // energized-and-parked: slow pulse
            else if (st.loopRunning) green = true;                 // running/online: solid
            else if (st.masterOp)    amber = true;                 // ready (OP, stopped)
            // else all off -- offline/idle shows a dark panel. parked/homing
            // are gated on masterOp above: motion state outlives a deinit,
            // and an idle Pi must not pulse green forever on a dead bus.

            gpiod_line_request_set_value(req, m_pins.ledFault, boolToVal(red));
            gpiod_line_request_set_value(req, m_pins.ledReady, boolToVal(amber));
            gpiod_line_request_set_value(req, m_pins.ledRun,   boolToVal(green));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(POLL_MS));
    }
}

// LED self-test sequence (runs on the panel thread). Each step aborts early if
// the panel is stopping. Red=fault pin, Amber=ready pin, Green=run pin.
// While the test runs, the poll loop is not sampling inputs, so an E-STOP
// press is acted on only when the test ends. Inputs are level-sampled and the
// mushroom latches mechanically, so the press is delayed, never lost - and the
// software stop is a convenience anyway: the latch hard-wired to the drives is
// the safety device.
void GpioPanel::runLedTest(void* request)
{
    auto* req = static_cast<gpiod_line_request*>(request);
    const unsigned R = m_pins.ledFault, A = m_pins.ledReady, G = m_pins.ledRun;
    auto set  = [&](unsigned pin, bool on){ gpiod_line_request_set_value(req, pin, boolToVal(on)); };
    auto hold = [&](int ms){ for (int i = 0; i < ms / 10 && m_running.load(); ++i)
                                 std::this_thread::sleep_for(std::chrono::milliseconds(10)); };

    LOG_INFO("GpioPanel: LED self-test starting.");
    set(R,0); set(A,0); set(G,0);
    // 1) single 0.5 s pulses: red, amber, green
    set(R,1); hold(500); set(R,0);
    set(A,1); hold(500); set(A,0);
    set(G,1); hold(500); set(G,0);
    // 2) cumulative 1 s: red, red+amber, red+amber+green
    set(R,1);                 hold(1000);
    set(A,1);                 hold(1000);
    set(G,1);                 hold(1000);
    // 3) all off
    set(R,0); set(A,0); set(G,0); hold(300);
    // 4) three flashes of all
    for (int i = 0; i < 3; ++i)
    {
        set(R,1); set(A,1); set(G,1); hold(250);
        set(R,0); set(A,0); set(G,0); hold(250);
    }
    LOG_INFO("GpioPanel: LED self-test complete.");
    // Normal LED state is reasserted by the next poll cycle.
}

void GpioPanel::stop()
{
    if (m_running.exchange(false))
    {
        if (m_thread.joinable())
            m_thread.join();
    }

    if (m_request)
    {
        auto* req = static_cast<gpiod_line_request*>(m_request);
        if (m_pins.useLeds)
        {
            gpiod_line_request_set_value(req, m_pins.ledRun,   GPIOD_LINE_VALUE_INACTIVE);
            gpiod_line_request_set_value(req, m_pins.ledReady, GPIOD_LINE_VALUE_INACTIVE);
            gpiod_line_request_set_value(req, m_pins.ledFault, GPIOD_LINE_VALUE_INACTIVE);
        }
        gpiod_line_request_release(req);
        m_request = nullptr;
    }
    if (m_chip)
    {
        gpiod_chip_close(static_cast<gpiod_chip*>(m_chip));
        m_chip = nullptr;
    }
}
