// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// DirectInputButtons - Windows (DirectInput8) capture/reader backend for the
// web binding wizard. The PC counterpart to pi/EvdevButtons; it implements the
// same three WebServer::ButtonBackendHooks (armCapture / pollCapture /
// bindingsChanged), reads the same web-owned buttons.json, and POSTs a mapped
// press to the local web server - so a physical button on the PC travels the
// exact same tested contract path as on the Pi.
//
// A single thread owns all DirectInput state. Idle (no bindings AND no capture
// armed) it BLOCKS on a condition variable - zero device work. When bindings
// exist or a capture is armed it polls attached game-controller devices at
// ~66 Hz for a fresh button-press edge. DirectInput is poll-based (no evdev-style
// blocking read), so this is a light poll gated on "actually in use".
//
// Identity is vendor:product (HID VID/PID, lower-case hex4, same shape as evdev)
// and the button "code" is the DirectInput button index (0..127). Bindings are
// per-host anyway (buttons.json lives beside config.json), so codes never need
// to match across platforms.
// ============================================================
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

class DirectInputButtons
{
public:
    ~DirectInputButtons() { stop(); }

    // buttonsJsonPath: the web-owned buttons.json; localWebPort: our own web
    // server (commands POST to 127.0.0.1:port/api/<cmd>).
    void start(const std::string& buttonsJsonPath, int localWebPort);
    void stop();

    // WebServer::ButtonBackendHooks targets.
    bool armCapture();
    std::string pollCapture();   // {"captured":false,...} or {"captured":true,...}
    void bindingsChanged();      // buttons.json saved -> hot-apply (reload)

    // Public so the file-scope DirectInput enumeration callback can build them.
    struct Dev
    {
        void*         device = nullptr;   // IDirectInputDevice8A*
        std::string   vendor, product, name;
        unsigned char prev[128] = {};     // previous rgbButtons snapshot (edge detect)
        bool          acquired = false;
    };

private:
    struct Binding { std::string cmd, vendor, product; int code = -1; };

    void threadMain();
    void loadBindings();                  // parse buttons.json (Qt JSON)
    void rescanDevices(bool wantAll);     // (re)open candidates (all vs bound-only)
    void closeDevices();
    void execCommand(const std::string& cmd);
    bool handlePress(const Dev& d, int code);   // returns true if consumed by capture

    std::string m_path;
    int         m_port = 8080;

    std::thread             m_thread;
    std::mutex              m_mu;         // guards bindings/capture/flags below
    std::condition_variable m_cv;
    std::atomic<bool>       m_stop{false};

    std::vector<Binding> m_bindings;      // under m_mu
    bool   m_reload         = false;      // under m_mu
    bool   m_captureArmed   = false;      // under m_mu
    double m_captureArmedAt = 0.0;        // monotonic secs, under m_mu
    std::string m_captured;               // last capture JSON, under m_mu

    void* m_di   = nullptr;               // IDirectInput8A* (thread-owned)
    void* m_hwnd = nullptr;               // hidden window for the cooperative-level anchor
    std::vector<Dev> m_devs;              // thread-local to threadMain
    bool  m_devsAreAll = false;           // current open-set mode
};
