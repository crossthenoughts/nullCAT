// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// EvdevButtons.cpp - see EvdevButtons.h. Linux-only (evdev).
// ============================================================
#include "EvdevButtons.h"
#include "Logging.h"
#include "Config.h"     // isBindableCommand - defense in depth at exec time
#include "httplib.h"

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>

#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <ctime>
#include <cstdio>
#include <cstring>
#include <linux/input.h>

static double monoSec()
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static std::string hex4(unsigned v)
{
    char b[8];
    std::snprintf(b, sizeof(b), "%04x", v & 0xFFFF);
    return b;
}

// Joystick/gamepad-class buttons: BTN_JOYSTICK..BTN_THUMBR and the
// TRIGGER_HAPPY range big DIY boxes spill into.
static bool isPadButton(int code)
{
    return (code >= BTN_JOYSTICK && code <= BTN_THUMBR)
        || (code >= BTN_TRIGGER_HAPPY && code <= BTN_TRIGGER_HAPPY40);
}

static bool hasPadButtons(int fd)
{
    unsigned long bits[(KEY_MAX / (8 * sizeof(long))) + 1] = {};
    if (ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(bits)), bits) < 0) return false;
    auto test = [&](int c){ return (bits[c / (8 * sizeof(long))] >> (c % (8 * sizeof(long)))) & 1UL; };
    for (int c = BTN_JOYSTICK; c <= BTN_THUMBR; ++c) if (test(c)) return true;
    for (int c = BTN_TRIGGER_HAPPY; c <= BTN_TRIGGER_HAPPY40; ++c) if (test(c)) return true;
    return false;
}

void EvdevButtons::start(const std::string& buttonsJsonPath, int localWebPort)
{
    m_path = buttonsJsonPath;
    m_port = localWebPort;
    loadBindings();
    m_stop = false;
    m_thread = std::thread(&EvdevButtons::threadMain, this);
    LOG_INFO("EvdevButtons: backend started (thread sleeps unless bindings exist or capture armed).");
}

void EvdevButtons::stop()
{
    m_stop = true;
    m_cv.notify_all();
    if (m_thread.joinable()) m_thread.join();
    closeDevices();
}

bool EvdevButtons::armCapture()
{
    std::lock_guard<std::mutex> lk(m_mu);
    m_captureArmed   = true;
    m_captureArmedAt = monoSec();
    m_captured.clear();
    m_cv.notify_all();
    return true;
}

std::string EvdevButtons::pollCapture()
{
    std::lock_guard<std::mutex> lk(m_mu);
    if (!m_captured.empty()) return m_captured;
    return m_captureArmed ? "{\"captured\":false,\"listening\":true}"
                          : "{\"captured\":false,\"listening\":false}";
}

void EvdevButtons::bindingsChanged()
{
    { std::lock_guard<std::mutex> lk(m_mu); m_reload = true; }
    m_cv.notify_all();
}

void EvdevButtons::loadBindings()
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
    LOG_INFO(strf("EvdevButtons: %zu binding(s) loaded.", m_bindings.size()));
}

void EvdevButtons::closeDevices()
{
    for (Dev& d : m_devs)
        if (d.fd >= 0) { if (d.grabbed) ioctl(d.fd, EVIOCGRAB, 0); ::close(d.fd); }
    m_devs.clear();
}

void EvdevButtons::rescanDevices(bool wantAll)
{
    closeDevices();
    m_devsAreAll = wantAll;

    std::vector<Binding> binds;
    { std::lock_guard<std::mutex> lk(m_mu); binds = m_bindings; }

    DIR* dir = opendir("/dev/input");
    if (!dir) return;
    while (dirent* e = readdir(dir))
    {
        if (std::strncmp(e->d_name, "event", 5) != 0) continue;
        std::string p = std::string("/dev/input/") + e->d_name;
        int fd = ::open(p.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) continue;
        if (!hasPadButtons(fd)) { ::close(fd); continue; }

        input_id id{};
        ioctl(fd, EVIOCGID, &id);
        char name[128] = {};
        ioctl(fd, EVIOCGNAME(sizeof(name) - 1), name);

        Dev d;
        d.fd = fd;
        d.vendor  = hex4(id.vendor);
        d.product = hex4(id.product);
        d.name    = name;

        bool bound = false;
        for (const Binding& b : binds)
            if (b.vendor == d.vendor && b.product == d.product) { bound = true; break; }

        if (!wantAll && !bound) { ::close(fd); continue; }

        // Exclusive grab for BOUND devices during runtime: presses cannot
        // leak anywhere else on the Pi. Capture-only devices are not grabbed.
        if (bound && !wantAll)
            d.grabbed = (ioctl(fd, EVIOCGRAB, 1) == 0);
        m_devs.push_back(d);
    }
    closedir(dir);
}

void EvdevButtons::execCommand(const std::string& cmd)
{
    if (!Config::isBindableCommand(cmd))   // defense in depth
    {
        LOG_WARNING(strf("EvdevButtons: refusing non-bindable command '%s'.", cmd.c_str()));
        return;
    }
    httplib::Client cli("127.0.0.1", m_port);
    cli.set_read_timeout(5, 0);
    auto r = cli.Post(("/api/" + cmd).c_str(), "", "application/json");
    LOG_INFO(strf("EvdevButtons: button -> %s => %s", cmd.c_str(),
                  r ? r->body.c_str() : "(no response)"));
}

void EvdevButtons::threadMain()
{
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

        // Rescan every 2s (hotplug) or when the open-set mode must change.
        if (monoSec() - lastScan > 2.0 || m_devsAreAll != armed)
        {
            rescanDevices(armed);
            lastScan = monoSec();
        }

        // Wait for events.
        std::vector<pollfd> pfds;
        for (Dev& d : m_devs) pfds.push_back({d.fd, POLLIN, 0});
        if (pfds.empty()) { usleep(200000); continue; }
        if (::poll(pfds.data(), pfds.size(), 500) <= 0) continue;

        for (size_t i = 0; i < pfds.size(); ++i)
        {
            if (!(pfds[i].revents & POLLIN)) continue;
            Dev& d = m_devs[i];
            input_event ev{};
            while (::read(d.fd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev))
            {
                if (ev.type != EV_KEY || ev.value != 1 || !isPadButton(ev.code)) continue;

                bool doCapture = false;
                {
                    std::lock_guard<std::mutex> lk(m_mu);
                    if (m_captureArmed)
                    {
                        m_captureArmed = false;
                        m_captured = std::string("{\"captured\":true")
                            + ",\"vendor\":\""  + d.vendor  + "\""
                            + ",\"product\":\"" + d.product + "\""
                            + ",\"code\":"      + std::to_string(ev.code)
                            + ",\"label\":\""   + d.name + " #" + std::to_string(ev.code) + "\"}";
                        doCapture = true;
                    }
                }
                if (doCapture)
                {
                    LOG_INFO(strf("EvdevButtons: captured %s:%s code=%d (%s)",
                                  d.vendor.c_str(), d.product.c_str(), ev.code, d.name.c_str()));
                    continue;
                }

                std::string cmd;
                {
                    std::lock_guard<std::mutex> lk(m_mu);
                    for (const Binding& b : m_bindings)
                        if (b.vendor == d.vendor && b.product == d.product && b.code == ev.code)
                        { cmd = b.cmd; break; }
                }
                if (!cmd.empty()) execCommand(cmd);
            }
        }
    }
    closeDevices();
}
