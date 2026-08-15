// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// EvdevButtons - Pi capture/reader backend for the web binding wizard.
//
// A thread INSIDE the existing nullcat-pi service rather than a standalone
// binder process: the wizard needs the WebServer and the reader in one
// conversation (capture flow), and a thread in the already-resident rig
// service adds ZERO new processes - nothing beyond the rig service itself
// may stay resident on the Pi. When there are no bindings and no capture
// armed, the thread BLOCKS on a condition variable: no device scanning, no
// polling, nothing.
//
// Capture: armCapture() opens every joystick-class /dev/input/event* and
// reports the next button press (vendor/product/code/name), then releases
// them. Auto-disarms after 15s if nothing is pressed.
// Runtime: bound devices (matched by vendor:product) are opened + EVIOCGRAB'd
// (exclusive - presses can't leak to anything else on the Pi); a mapped press
// POSTs to the local web server, so every physical button travels the same
// tested contract path as every other surface.
// ============================================================
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

class EvdevButtons
{
public:
    // buttonsJsonPath: the web-owned buttons.json; localWebPort: our own
    // web server (commands are POSTed to 127.0.0.1:port/api/<cmd>).
    void start(const std::string& buttonsJsonPath, int localWebPort);
    void stop();

    // WebServer::ButtonBackendHooks targets.
    bool armCapture();
    std::string pollCapture();   // {"captured":false} or {"captured":true,...}
    void bindingsChanged();      // buttons.json saved -> hot-apply (reload)

private:
    struct Binding { std::string cmd, vendor, product; int code = -1; };
    struct Dev { int fd = -1; std::string vendor, product, name; bool grabbed = false; };

    void threadMain();
    void loadBindings();                  // parse buttons.json (Qt JSON)
    void rescanDevices(bool wantAll);     // open candidates (all vs bound-only)
    void closeDevices();
    void execCommand(const std::string& cmd);

    std::string m_path;
    int         m_port = 8080;

    std::thread             m_thread;
    std::mutex              m_mu;         // guards bindings/capture/flags below
    std::condition_variable m_cv;
    std::atomic<bool>       m_stop{false};

    std::vector<Binding> m_bindings;      // under m_mu
    bool   m_reload = false;              // under m_mu
    bool   m_captureArmed = false;        // under m_mu
    double m_captureArmedAt = 0.0;        // CLOCK_MONOTONIC secs, under m_mu
    std::string m_captured;               // last capture JSON, under m_mu

    std::vector<Dev> m_devs;              // thread-local to threadMain
    bool m_devsAreAll = false;            // current open-set mode
};
