// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// GpioPanel.h - Pi-only physical control panel (libgpiod v2)
//
// A small wired panel: 3 inputs (E-STOP mushroom NC, ENGAGE, PARK) and
// 3 status LEDs (Run / Ready / Fault). A low-priority poll thread on
// cores 0/1 (never the isolated RT cores) debounces the buttons, drives
// the LEDs from a status snapshot, and fires the SAME controller actions
// the WebServer uses - passed in as callbacks, so there is no second copy
// of the control logic and no dependency on the web server.
//
// The panel's e-stop path is software convenience only: the mushroom is
// mechanically latched and hard-wired into the drives' stop chain, and
// that hardware latch - not this code - is the safety device.
//
// Compiled only when libgpiod is found (HAVE_LIBGPIOD; see pi/CMakeLists.txt
// and setup.sh → libgpiod-dev). gpiod.h is kept out of this header (opaque
// handles) so main_headless can include it unconditionally.
// ============================================================

#include <atomic>
#include <thread>
#include <functional>
#include <string>

// Snapshot the panel reads each cycle to drive the LEDs.
struct PanelStatus
{
    bool masterOp    = false;   // EtherCAT operational
    bool loopRunning = false;   // control loop active
    bool estop       = false;   // software e-stop active
    bool fault       = false;   // any drive fault/fatal
    bool initBusy    = false;   // init/de-init in progress
    bool homing      = false;   // any axis homing
    bool parked      = false;   // all axes parked
};

// Button → action callbacks. Wired in main_headless to the same calls the
// web endpoints make.
struct PanelActions
{
    std::function<void()> init;          // ENGAGE when offline
    std::function<void()> start;         // ENGAGE when ready (OP, stopped)
    std::function<void()> stop;          // ENGAGE when running (stop loop; never de-inits)
    std::function<void()> park;          // PARK when running (not parked)
    std::function<void()> unpark;        // PARK when already parked (toggle)
    std::function<void()> estop;         // mushroom opened
    std::function<void()> estopRelease;  // mushroom closed again (auto-release)
};

// BCM line numbers (and the chip name). Loaded from config.json.
// useLeds / useButtons gate which lines are requested, so unused pins stay
// free for other purposes (set from gpioMode in main_headless).
struct PanelPins
{
    std::string chip     = "gpiochip0";
    unsigned    estop    = 17;   // input, NC mushroom (open = e-stop)  [always]
    unsigned    engage   = 27;   // input, momentary to GND             [useButtons]
    unsigned    park     = 22;   // input, momentary to GND             [useButtons]
    unsigned    ledRun   = 23;   // output, green                       [useLeds]
    unsigned    ledReady = 24;   // output, amber                       [useLeds]
    unsigned    ledFault = 25;   // output, red                         [useLeds]
    bool        useLeds    = true;
    bool        useButtons = true;
};

class GpioPanel
{
public:
    GpioPanel() = default;
    ~GpioPanel();

    GpioPanel(const GpioPanel&)            = delete;
    GpioPanel& operator=(const GpioPanel&) = delete;

    // Request the lines and start the poll thread. Returns false (logged) if
    // the chip/lines cannot be opened - the app then runs without a panel.
    bool start(const PanelPins& pins,
               std::function<PanelStatus()> getStatus,
               const PanelActions& actions);

    void stop();

    // Request an LED self-test sequence (runs on the panel thread next cycle).
    // No-op if the active mode has no LEDs.
    void requestLedTest() { m_ledTestReq.store(true); }

private:
    void run();
    void runLedTest(void* request);

    PanelPins                    m_pins;
    std::function<PanelStatus()> m_getStatus;
    PanelActions                 m_actions;

    std::thread       m_thread;
    std::atomic<bool> m_running{false};
    std::atomic<bool> m_ledTestReq{false};

    // Opaque libgpiod handles (defined in the .cpp to keep gpiod.h private).
    void* m_chip    = nullptr;   // gpiod_chip*
    void* m_request = nullptr;   // gpiod_line_request*
};
