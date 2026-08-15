// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// DirectInputButtons.cpp - see DirectInputButtons.h. Windows-only (DirectInput8).
// The Windows counterpart to pi/EvdevButtons.cpp: same three hooks, same
// buttons.json contract, same POST-to-local-webserver runtime path.
// ============================================================
#include "DirectInputButtons.h"

#ifdef _WIN32   // whole backend is Windows-only; compiles to nothing elsewhere

#include "Logging.h"
#include "Config.h"     // isBindableCommand - defense in depth at exec time

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>
#include <QString>

#ifndef WIN32_LEAN_AND_MEAN
#  define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#  define NOMINMAX
#endif
#define DIRECTINPUT_VERSION 0x0800
#include <winsock2.h>    // before windows.h so httplib's winsock2 stays clean
#include <windows.h>
#include <dinput.h>
#include "httplib.h"

#include <chrono>
#include <cstdio>
#include <cstring>

#pragma comment(lib, "dinput8.lib")
#pragma comment(lib, "dxguid.lib")

static double monoSec() { return GetTickCount64() / 1000.0; }

static std::string hex4(unsigned v)
{
    char b[8];
    std::snprintf(b, sizeof(b), "%04x", v & 0xFFFF);
    return b;
}

// ---- device enumeration ----------------------------------------------------
// EnumDevices hands us each attached game controller. We collect just its
// instance GUID + HID identity here; the actual IDirectInputDevice8 is created
// in rescanDevices (which knows the bound-vs-capture open-set policy).
namespace {
struct EnumHit { GUID guid; std::string vendor, product, name; };
struct EnumCtx { std::vector<EnumHit>* hits; };

BOOL WINAPI enumCb(const DIDEVICEINSTANCEA* inst, void* pv)
{
    auto* ctx = static_cast<EnumCtx*>(pv);
    EnumHit h;
    h.guid = inst->guidInstance;
    // For HID controllers guidProduct.Data1 == MAKELONG(vendorId, productId).
    const unsigned long vp = inst->guidProduct.Data1;
    h.vendor  = hex4(vp & 0xFFFF);
    h.product = hex4((vp >> 16) & 0xFFFF);
    h.name    = inst->tszProductName ? inst->tszProductName : "";
    ctx->hits->push_back(std::move(h));
    return DIENUM_CONTINUE;
}
} // namespace

// ---- lifecycle -------------------------------------------------------------
void DirectInputButtons::start(const std::string& buttonsJsonPath, int localWebPort)
{
    m_path = buttonsJsonPath;
    m_port = localWebPort;
    loadBindings();
    m_stop = false;
    m_thread = std::thread(&DirectInputButtons::threadMain, this);
    LOG_INFO("DirectInputButtons: backend started (thread sleeps unless bindings exist or capture armed).");
}

void DirectInputButtons::stop()
{
    m_stop = true;
    m_cv.notify_all();
    if (m_thread.joinable()) m_thread.join();
}

// ---- hooks -----------------------------------------------------------------
bool DirectInputButtons::armCapture()
{
    std::lock_guard<std::mutex> lk(m_mu);
    m_captureArmed   = true;
    m_captureArmedAt = monoSec();
    m_captured.clear();
    m_cv.notify_all();
    return true;
}

std::string DirectInputButtons::pollCapture()
{
    std::lock_guard<std::mutex> lk(m_mu);
    if (!m_captured.empty()) return m_captured;
    return m_captureArmed ? "{\"captured\":false,\"listening\":true}"
                          : "{\"captured\":false,\"listening\":false}";
}

void DirectInputButtons::bindingsChanged()
{
    { std::lock_guard<std::mutex> lk(m_mu); m_reload = true; }
    m_cv.notify_all();
}

// ---- bindings --------------------------------------------------------------
void DirectInputButtons::loadBindings()
{
    std::vector<Binding> fresh;
    QFile f(QString::fromStdString(m_path));
    if (f.open(QIODevice::ReadOnly))
    {
        QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
        for (const QJsonValue& v : doc.object().value("bindings").toArray())
        {
            QJsonObject o = v.toObject();
            Binding b;
            b.cmd     = o.value("cmd").toString().toStdString();
            b.vendor  = o.value("vendor").toString().toLower().toStdString();
            b.product = o.value("product").toString().toLower().toStdString();
            b.code    = o.value("code").toInt(-1);
            if (!b.cmd.empty() && b.code >= 0) fresh.push_back(b);
        }
    }
    std::lock_guard<std::mutex> lk(m_mu);
    m_bindings = std::move(fresh);
    LOG_INFO(strf("DirectInputButtons: %zu binding(s) loaded.", m_bindings.size()));
}

// ---- device management -----------------------------------------------------
void DirectInputButtons::closeDevices()
{
    for (Dev& d : m_devs)
    {
        auto* dev = static_cast<IDirectInputDevice8A*>(d.device);
        if (dev) { if (d.acquired) dev->Unacquire(); dev->Release(); }
    }
    m_devs.clear();
}

void DirectInputButtons::rescanDevices(bool wantAll)
{
    // Diff-based: keep already-open handles that are still present and still
    // wanted; only open newly-appeared devices and close vanished/unwanted ones.
    // Tearing down and re-creating every COM handle each scan (Release/
    // CreateDevice churn plus the full HID enumeration, repeating during live
    // motion) shows up as RT jitter -- so the steady state must do nothing
    // but the (light) EnumDevices walk.
    m_devsAreAll = wantAll;

    std::vector<Binding> binds;
    { std::lock_guard<std::mutex> lk(m_mu); binds = m_bindings; }

    auto* di = static_cast<IDirectInput8A*>(m_di);
    if (!di) return;

    std::vector<EnumHit> hits;
    EnumCtx ctx{ &hits };
    di->EnumDevices(DI8DEVCLASS_GAMECTRL, &enumCb, &ctx, DIEDFL_ATTACHEDONLY);

    // wanted(): capture opens every controller; runtime opens only bound ones.
    // Identity is vendor+product -- the same key bindings match on, so two
    // identical controllers are indistinguishable here as they are in a binding.
    auto wanted = [&](const EnumHit& h) {
        if (wantAll) return true;
        for (const Binding& b : binds)
            if (b.vendor == h.vendor && b.product == h.product) return true;
        return false;
    };

    const size_t before = m_devs.size();

    // Close devices that vanished or are no longer wanted.
    for (size_t i = 0; i < m_devs.size(); )
    {
        bool keep = false;
        for (const EnumHit& h : hits)
            if (h.vendor == m_devs[i].vendor && h.product == m_devs[i].product && wanted(h)) { keep = true; break; }
        if (keep) { ++i; continue; }
        auto* dev = static_cast<IDirectInputDevice8A*>(m_devs[i].device);
        if (dev) { if (m_devs[i].acquired) dev->Unacquire(); dev->Release(); }
        m_devs.erase(m_devs.begin() + static_cast<long>(i));
    }

    // Open newly-appeared wanted devices that aren't already open.
    for (EnumHit& h : hits)
    {
        if (!wanted(h)) continue;
        bool already = false;
        for (const Dev& d : m_devs)
            if (d.vendor == h.vendor && d.product == h.product) { already = true; break; }
        if (already) continue;

        IDirectInputDevice8A* dev = nullptr;
        if (FAILED(di->CreateDevice(h.guid, &dev, nullptr))) continue;
        if (FAILED(dev->SetDataFormat(&c_dfDIJoystick2))) { dev->Release(); continue; }
        // Background + non-exclusive: read presses even when the Qt window isn't
        // focused (the user is in a game), without stealing them from anything.
        dev->SetCooperativeLevel(static_cast<HWND>(m_hwnd),
                                 DISCL_BACKGROUND | DISCL_NONEXCLUSIVE);
        Dev d;
        d.device   = dev;
        d.vendor   = h.vendor;
        d.product  = h.product;
        d.name     = h.name;
        d.acquired = SUCCEEDED(dev->Acquire());
        m_devs.push_back(std::move(d));
    }

    // Log only when the open-set changed -- no steady-state spam.
    if (m_devs.size() != before)
        LOG_INFO(strf("DirectInputButtons: open device set -> %zu device(s) (%s).",
                      m_devs.size(), wantAll ? "capture: all" : "runtime: bound-only"));
}

// ---- press handling --------------------------------------------------------
bool DirectInputButtons::handlePress(const Dev& d, int code)
{
    std::lock_guard<std::mutex> lk(m_mu);
    if (!m_captureArmed) return false;
    m_captureArmed = false;
    m_captured = std::string("{\"captured\":true")
        + ",\"vendor\":\""  + d.vendor  + "\""
        + ",\"product\":\"" + d.product + "\""
        + ",\"code\":"      + std::to_string(code)
        + ",\"label\":\""   + d.name + " #" + std::to_string(code) + "\"}";
    LOG_INFO(strf("DirectInputButtons: captured %s:%s code=%d (%s)",
                  d.vendor.c_str(), d.product.c_str(), code, d.name.c_str()));
    return true;
}

void DirectInputButtons::execCommand(const std::string& cmd)
{
    if (!Config::isBindableCommand(cmd))   // defense in depth
    {
        LOG_WARNING(strf("DirectInputButtons: refusing non-bindable command '%s'.", cmd.c_str()));
        return;
    }
    httplib::Client cli("127.0.0.1", m_port);
    cli.set_read_timeout(5, 0);
    auto r = cli.Post(("/api/" + cmd).c_str(), "", "application/json");
    LOG_INFO(strf("DirectInputButtons: button -> %s => %s", cmd.c_str(),
                  r ? r->body.c_str() : "(no response)"));
}

// ---- worker thread ---------------------------------------------------------
void DirectInputButtons::threadMain()
{
    CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);   // DI prefers an STA; ignore mode-change

    // A hidden top-level window anchors DISCL_BACKGROUND. Never shown.
    WNDCLASSEXA wc{};
    wc.cbSize        = sizeof(wc);
    wc.lpfnWndProc   = DefWindowProcA;
    wc.hInstance     = GetModuleHandleA(nullptr);
    wc.lpszClassName = "NullCatDIButtons";
    RegisterClassExA(&wc);   // harmless if already registered
    m_hwnd = CreateWindowExA(0, "NullCatDIButtons", "nullCAT input", WS_OVERLAPPED,
                             0, 0, 0, 0, nullptr, nullptr, GetModuleHandleA(nullptr), nullptr);

    IDirectInput8A* di = nullptr;
    if (FAILED(DirectInput8Create(GetModuleHandleA(nullptr), DIRECTINPUT_VERSION,
                                  IID_IDirectInput8A, reinterpret_cast<void**>(&di), nullptr)))
    {
        LOG_WARNING("DirectInputButtons: DirectInput8Create failed -- button capture unavailable.");
        if (m_hwnd) { DestroyWindow(static_cast<HWND>(m_hwnd)); m_hwnd = nullptr; }
        CoUninitialize();
        return;
    }
    m_di = di;

    double lastScan = 0.0;
    while (!m_stop.load())
    {
        bool armed, haveBindings, reload;
        {
            std::unique_lock<std::mutex> lk(m_mu);
            // Idle = no bindings and no capture: BLOCK. Zero work while unused.
            m_cv.wait(lk, [&]{ return m_stop.load() || m_reload || m_captureArmed || !m_bindings.empty(); });
            if (m_stop.load()) break;
            reload = m_reload; m_reload = false;
            // Capture auto-disarm: nothing pressed within 15s.
            if (m_captureArmed && monoSec() - m_captureArmedAt > 15.0)
                m_captureArmed = false;
            armed        = m_captureArmed;
            haveBindings = !m_bindings.empty();
        }
        if (reload) { loadBindings(); lastScan = 0.0; }
        if (!armed && !haveBindings) { closeDevices(); m_devsAreAll = false; continue; }

        // Hotplug rescan every 5s (a button box is plugged in once, so this
        // doesn't need to be fast) or immediately when the open-set mode changes
        // (runtime bound-only <-> capture all). rescanDevices() is diff-based,
        // so a no-change scan does no COM work.
        if (monoSec() - lastScan > 5.0 || m_devsAreAll != armed)
        {
            rescanDevices(armed);
            lastScan = monoSec();
        }

        // Poll each device for fresh press edges. DirectInput is poll-based.
        for (Dev& d : m_devs)
        {
            auto* dev = static_cast<IDirectInputDevice8A*>(d.device);
            if (!dev) continue;
            if (!d.acquired) { d.acquired = SUCCEEDED(dev->Acquire()); if (!d.acquired) continue; }
            if (FAILED(dev->Poll())) dev->Acquire();   // lost focus/device -> reacquire

            DIJOYSTATE2 st{};
            if (FAILED(dev->GetDeviceState(sizeof(st), &st))) { d.acquired = false; continue; }

            for (int c = 0; c < 128; ++c)
            {
                const bool down    = (st.rgbButtons[c] & 0x80) != 0;
                const bool wasDown = d.prev[c] != 0;
                d.prev[c] = down ? 1 : 0;
                if (!down || wasDown) continue;        // only the press EDGE

                if (handlePress(d, c)) continue;       // consumed by capture

                std::string cmd;
                {
                    std::lock_guard<std::mutex> lk(m_mu);
                    for (const Binding& b : m_bindings)
                        if (b.vendor == d.vendor && b.product == d.product && b.code == c)
                        { cmd = b.cmd; break; }
                }
                if (!cmd.empty()) execCommand(cmd);
            }
        }

        // ~66 Hz poll cadence while active; wakes early on stop / reload.
        std::unique_lock<std::mutex> lk(m_mu);
        m_cv.wait_for(lk, std::chrono::milliseconds(15),
                      [&]{ return m_stop.load() || m_reload; });
    }

    closeDevices();
    if (m_di)   { static_cast<IDirectInput8A*>(m_di)->Release(); m_di = nullptr; }
    if (m_hwnd) { DestroyWindow(static_cast<HWND>(m_hwnd)); m_hwnd = nullptr; }
    CoUninitialize();
}

#endif // _WIN32
