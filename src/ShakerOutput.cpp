// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#include "ShakerOutput.h"

// miniaudio, playback only, the backends we ship for: ALSA (Pi), WASAPI
// (Windows) and the null backend (CI, no card). Everything else off to
// keep the build lean and the behaviour predictable.
#define MA_NO_DECODING
#define MA_NO_ENCODING
#define MA_NO_WAV
#define MA_NO_FLAC
#define MA_NO_MP3
#define MA_NO_RESOURCE_MANAGER
#define MA_NO_NODE_GRAPH
#define MA_NO_ENGINE
#define MA_NO_GENERATION
#define MA_NO_PULSEAUDIO
#define MA_NO_JACK
#define MA_NO_COREAUDIO
#define MA_NO_SNDIO
#define MA_NO_AUDIO4
#define MA_NO_OSS
#define MA_NO_AAUDIO
#define MA_NO_OPENSL
#define MA_NO_WEBAUDIO
#define MA_NO_CUSTOM
#define MA_NO_WINMM
#define MA_NO_DSOUND
#define MINIAUDIO_IMPLEMENTATION
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include "miniaudio.h"
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#if defined(_WIN32)
#undef CLSCTX_ALL   // miniaudio's own (23), the same value combaseapi.h defines
#include <mmdeviceapi.h>
#include <endpointvolume.h>
#endif

namespace haptics {

struct ShakerOutput::Impl
{
    ma_context context;
    bool       contextOk = false;
    ma_device  device;
    bool       deviceOk = false;
    bool         wasapi = false;   // the card is a Windows endpoint (its volume is held at full)
    std::wstring endpointId;       // ...this one ("" = the default output)
};

#if defined(_WIN32)
// A shaker card at full volume on Windows, as the Pi sets a card to full
// line level: the endpoint's master level, its mute and each channel's
// level. On a USB card the endpoint volume is the card's own hardware
// volume, which exclusive mode does not bypass, and another program
// (SimHub) was found binding a shaker card and muting it. id: the card's
// endpoint ("" = the default output). found: the master level it had (0..1, -1 = unread).
// Returns whether anything was changed. Any thread (COM is set up and
// released here when the thread has none).
static bool holdFullVolume(const std::wstring& id, float& found, bool& wasMuted)
{
    found = -1.0f; wasMuted = false;
    const HRESULT ci = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    bool changed = false;
    IMMDeviceEnumerator* en = nullptr; IMMDevice* dev = nullptr; IAudioEndpointVolume* vol = nullptr;
    if (SUCCEEDED(CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr, CLSCTX_ALL, __uuidof(IMMDeviceEnumerator),
                                   reinterpret_cast<void**>(&en))) && en)
    {
        const HRESULT hr = id.empty() ? en->GetDefaultAudioEndpoint(eRender, eConsole, &dev) : en->GetDevice(id.c_str(), &dev);
        if (SUCCEEDED(hr) && dev
            && SUCCEEDED(dev->Activate(__uuidof(IAudioEndpointVolume), CLSCTX_ALL, nullptr, reinterpret_cast<void**>(&vol))) && vol)
        {
            float m = 0.0f; BOOL mute = FALSE;
            if (SUCCEEDED(vol->GetMasterVolumeLevelScalar(&m))) found = m;
            if (SUCCEEDED(vol->GetMute(&mute)) && mute)
            {
                wasMuted = true;
                if (SUCCEEDED(vol->SetMute(FALSE, nullptr))) changed = true;
            }
            if (found >= 0.0f && found < 0.999f && SUCCEEDED(vol->SetMasterVolumeLevelScalar(1.0f, nullptr))) changed = true;
            UINT n = 0;
            if (SUCCEEDED(vol->GetChannelCount(&n)))
                for (UINT c = 0; c < n; ++c)
                {
                    float cv = 1.0f;
                    if (SUCCEEDED(vol->GetChannelVolumeLevelScalar(c, &cv)) && cv < 0.999f
                        && SUCCEEDED(vol->SetChannelVolumeLevelScalar(c, 1.0f, nullptr)))
                        changed = true;
                }
        }
    }
    if (vol) vol->Release();
    if (dev) dev->Release();
    if (en)  en->Release();
    if (SUCCEEDED(ci)) CoUninitialize();   // S_OK and S_FALSE are ours to balance; a thread already in another mode is not
    return changed;
}
#endif

ShakerOutput::ShakerOutput() : m_impl(new Impl)
{
    ma_backend backends[] = { ma_backend_wasapi, ma_backend_alsa, ma_backend_null };
    ma_context_config cfg = ma_context_config_init();
    // A test run (NULLCAT_NO_RT) keeps the audio thread at normal priority,
    // like every other thread it starts.
    const char* noRt = std::getenv("NULLCAT_NO_RT");
    if (noRt && noRt[0] && !(noRt[0] == '0' && noRt[1] == '\0')) cfg.threadPriority = ma_thread_priority_normal;
    m_impl->contextOk = (ma_context_init(backends, 3, &cfg, &m_impl->context) == MA_SUCCESS);
}

ShakerOutput::~ShakerOutput()
{
    close();
    if (m_impl->contextOk) ma_context_uninit(&m_impl->context);
    delete m_impl;
}

std::vector<ShakerDeviceInfo> ShakerOutput::listDevices() const
{
    std::vector<ShakerDeviceInfo> out;
    if (!m_impl->contextOk) return out;
    // The list is the context's own buffer, rewritten (and reallocated when
    // a card appears) by every call: open() reads it under this lock, and a
    // reopen from the page runs open() while the settings page lists.
    std::lock_guard<std::mutex> lk(m_mx);
    ma_device_info* playback = nullptr; ma_uint32 count = 0;
    if (ma_context_get_devices(&m_impl->context, &playback, &count, nullptr, nullptr) != MA_SUCCESS) return out;
    for (ma_uint32 i = 0; i < count; ++i)
        out.push_back({ playback[i].name, playback[i].isDefault != 0 });
    return out;
}

void ShakerOutput::dataCallback(void* device, void* out, const void* /*in*/, unsigned frames)
{
    ShakerOutput* self = static_cast<ShakerOutput*>(static_cast<ma_device*>(device)->pUserData);
    if (!self) return;
    self->m_callbacks.fetch_add(1, std::memory_order_relaxed);
    self->m_dsp.render(static_cast<float*>(out), static_cast<int>(frames));
}

// The backend's word on the device (its audio or notification thread):
// stopped by itself (WASAPI on a card unplugged or disabled; not our own
// close) or moved to another route (the default output changed under
// us). Either way the card we were asked for is no longer what plays:
// the gate closes, the status says why, reopen() picks it up again.
// Atomics only, nothing that can block the thread it runs on.
void ShakerOutput::notificationCallback(const void* notification)
{
    const ma_device_notification* n = static_cast<const ma_device_notification*>(notification);
    ShakerOutput* self = n && n->pDevice ? static_cast<ShakerOutput*>(n->pDevice->pUserData) : nullptr;
    if (!self) return;
    if (n->type == ma_device_notification_type_stopped && !self->m_closing.load(std::memory_order_acquire))
        self->markLost("stopped");
    else if (n->type == ma_device_notification_type_rerouted)
        self->markLost("rerouted");
}

static constexpr int kLostStopped = 1, kLostRerouted = 2, kLostQuiet = 3;

void ShakerOutput::markLost(const char* why)
{
    if (!m_open.load(std::memory_order_acquire)) return;
    const int kind = (std::strcmp(why, "rerouted") == 0) ? kLostRerouted
                   : (std::strcmp(why, "quiet")    == 0) ? kLostQuiet : kLostStopped;
    m_lost.store(kind, std::memory_order_release);
    m_open.store(false, std::memory_order_release);
}

void ShakerOutput::checkHealth()
{
    std::lock_guard<std::mutex> lk(m_mx);
    const int64_t nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
#if defined(_WIN32)
    // Windows or another program can turn a card down or mute it at any
    // time: back to full, at most once a second, counted for the status line.
    if (m_impl->wasapi && m_impl->deviceOk && m_open.load(std::memory_order_acquire) && nowMs - m_volumeCheckMs >= 1000)
    {
        m_volumeCheckMs = nowMs;
        float had = -1.0f; bool muted = false;
        if (holdFullVolume(m_impl->endpointId, had, muted)) m_volumeResets.fetch_add(1, std::memory_order_relaxed);
    }
#endif
    const uint64_t calls = m_callbacks.load(std::memory_order_relaxed);
    if (m_lost.load(std::memory_order_acquire) == kLostQuiet)
    {
        // A card marked quiet that is pulling data again only paused (the
        // machine slept, the backend caught its breath): back in play by
        // itself, nothing to open again.
        if (calls != m_healthCallbacks && m_impl->deviceOk)
        {
            m_healthMs = nowMs; m_healthCallbacks = calls;
            m_lost.store(0, std::memory_order_release);
            m_open.store(true, std::memory_order_release);
        }
        return;
    }
    if (!m_open.load(std::memory_order_acquire)) return;
    if (m_healthMs == 0 || calls != m_healthCallbacks) { m_healthMs = nowMs; m_healthCallbacks = calls; return; }
    if (nowMs - m_healthMs >= static_cast<int64_t>(kQuietSec * 1000.0)) markLost("quiet");
}

std::string ShakerOutput::error() const
{
    std::lock_guard<std::mutex> lk(m_mx);
    switch (m_lost.load(std::memory_order_acquire))
    {
        case kLostStopped:  return "the sound card stopped (unplugged or disabled?): those shakers are silent until it is opened again";
        case kLostRerouted: return "the sound output moved to another device: those shakers are silent until it is opened again";
        case kLostQuiet:    return "the sound card has gone quiet (no audio for 2 s; unplugged?): those shakers are silent until it is opened again";
        default:            return m_error;
    }
}

#if defined(__linux__)
// A shaker card plays at full line level: the volume belongs on the amps and
// the route gains, and many USB boxes come up at about -24 dB (a bench rig's
// shakers were near silent). Output controls only, playback direction only:
// Mic and Line also have a playback path (input monitoring) and are left
// alone. Runs once per open, never on the RT path; a card without one of
// these controls (or no amixer) is simply left as it is.
static void fullLineLevel(const std::string& alsaId)
{
    // hw:CARD=NAME,DEV=0 or hw:0,0 -> the card name or number.
    const size_t colon = alsaId.find(':');
    if (colon == std::string::npos) return;
    std::string card = alsaId.substr(colon + 1);
    if (card.rfind("CARD=", 0) == 0) card = card.substr(5);
    card = card.substr(0, card.find(','));
    if (card.empty() || card.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-") != std::string::npos)
        return;   // only plain names reach the shell
    static const char* const kOutputs[] = { "Master", "Speaker", "PCM", "Headphone", "Front",
                                            "Surround", "Center", "LFE", "Side", "Rear" };
    for (const char* c : kOutputs)
    {
        const std::string cmd = "amixer -q -c '" + card + "' sset '" + c + "' playback 100% unmute >/dev/null 2>&1";
        (void)std::system(cmd.c_str());
    }
}
#endif

bool ShakerOutput::open(const std::string& deviceName, int channels, double controlHz, int sampleRate)
{
    std::lock_guard<std::mutex> lk(m_mx);
    closeLocked();
    m_error.clear();
    m_lost.store(0, std::memory_order_release);
    m_healthMs = 0; m_healthCallbacks = 0;
    m_reqDevice = deviceName; m_reqChannels = channels; m_reqControlHz = controlHz; m_reqRate = sampleRate; m_reqValid = true;
    std::string fallbackReason;   // why a shared device was opened instead of the raw one (empty = it was not)
    if (!m_impl->contextOk) { m_error = "no audio backend available"; return false; }
    channels = std::max(1, std::min(MAX_SHAKERS, channels));

    ma_device_config cfg = ma_device_config_init(ma_device_type_playback);
    cfg.playback.format   = ma_format_f32;
    cfg.playback.channels = static_cast<ma_uint32>(channels);
    cfg.sampleRate        = static_cast<ma_uint32>(sampleRate);
    cfg.dataCallback      = reinterpret_cast<ma_device_data_proc>(&ShakerOutput::dataCallback);
    cfg.notificationCallback = reinterpret_cast<ma_device_notification_proc>(&ShakerOutput::notificationCallback);
    cfg.pUserData         = this;
    cfg.periodSizeInFrames = 240;    // 5 ms at 48 kHz
    cfg.periods            = 3;

    // Device by name (substring, case-insensitive), "" = default, "null" = the null backend.
    // On ALSA the card itself is miniaudio's ":card,dev" id (or "hw:..."),
    // listed beside shared names with the same description (default:,
    // sysdefault:, front:, ...): prefer the card itself, then plughw:, then
    // whatever else matched.
    ma_device_id chosen; bool haveId = false;
    // "null:N": the null backend as a pretend card with N channels of its own.
    const bool wantNull = (deviceName == "null") || (deviceName.rfind("null:", 0) == 0);
    const int  nullOwn  = (wantNull && deviceName.size() > 5) ? std::atoi(deviceName.c_str() + 5) : 0;
    const bool alsa = (m_impl->context.backend == ma_backend_alsa);
    const bool wasapi = !wantNull && (m_impl->context.backend == ma_backend_wasapi);
    if (!deviceName.empty() && !wantNull)
    {
        ma_device_info* playback = nullptr; ma_uint32 count = 0;
        if (ma_context_get_devices(&m_impl->context, &playback, &count, nullptr, nullptr) == MA_SUCCESS)
        {
            std::string want = deviceName;
            for (char& c : want) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            int bestRank = 99;
            for (ma_uint32 i = 0; i < count; ++i)
            {
                std::string have = playback[i].name;
                for (char& c : have) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                std::string id = alsa ? std::string(playback[i].id.alsa) : std::string();
                for (char& c : id) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                if (have.find(want) == std::string::npos && (id.empty() || id.find(want) == std::string::npos)) continue;
                const int rank = !alsa ? 2 : (id.rfind(":", 0) == 0 || id.rfind("hw:", 0) == 0) ? 0
                               : (id.rfind("plughw:", 0) == 0) ? 1 : 2;
                if (rank < bestRank) { chosen = playback[i].id; haveId = true; bestRank = rank; }
            }
        }
        if (!haveId) { m_error = "audio device not found: " + deviceName; return false; }
        cfg.playback.pDeviceID = &chosen;
    }
    m_impl->wasapi = wasapi;
    m_impl->endpointId.clear();
#if defined(_WIN32)
    if (wasapi && haveId) m_impl->endpointId = std::wstring(chosen.wasapi);
#endif
    // Ask for the card exclusively. On ALSA in shared mode miniaudio opens
    // its "dmix" layer first (two channels, a 16384-frame buffer: eight
    // shakers squeezed into stereo and 60 ms or more late; found on the
    // bench), exclusively the card itself ("hw": every channel, the periods
    // asked for). On Windows, exclusively the card plays without the
    // Windows mixer, the app's volume or other programs' sound, in its own
    // format (the classic Sound panel's speaker setup and default format)
    // and with a short buffer. A card another program holds, or one that
    // refuses, falls back to shared below.
    if ((alsa || wasapi) && !wantNull) cfg.playback.shareMode = ma_share_mode_exclusive;

    ma_result r;
    if (wantNull)
    {
        // A context of its own on the null backend, so "null" works even
        // where the shared context picked a real backend.
        ma_backend nb[] = { ma_backend_null };
        ma_context_config cc = ma_context_config_init();
        static ma_context nullCtx; static bool nullCtxOk = false;
        if (!nullCtxOk) nullCtxOk = (ma_context_init(nb, 1, &cc, &nullCtx) == MA_SUCCESS);
        if (!nullCtxOk) { m_error = "null backend unavailable"; return false; }
        r = ma_device_init(&nullCtx, &cfg, &m_impl->device);
    }
    else
    {
        r = ma_device_init(&m_impl->context, &cfg, &m_impl->device);
        // The card busy (another program holds it): the shared layer, with
        // the reason kept for the status line.
        if (r != MA_SUCCESS && cfg.playback.shareMode == ma_share_mode_exclusive)
        {
            fallbackReason = std::string("card not available on its own (") + ma_result_description(r) + "), "
                           + (wasapi ? "shared through Windows: its mixer and volumes apply"
                                     : "shared: 2 channels, high latency");
            cfg.playback.shareMode = ma_share_mode_shared;
            r = ma_device_init(&m_impl->context, &cfg, &m_impl->device);
        }
    }
    if (r != MA_SUCCESS) { m_error = std::string("audio device open failed: ") + ma_result_description(r); return false; }
    m_impl->deviceOk = true;

    m_sampleRate = static_cast<int>(m_impl->device.sampleRate);
    m_dsp.configure(controlHz, m_sampleRate, static_cast<int>(m_impl->device.playback.channels));
    // What the card itself has (its native channel count) as against what
    // was asked for: miniaudio renders the asked-for count and folds the
    // rest into the card's channels, so a stereo card asked for four would
    // play shakers 3 and 4 into 1 and 2. push() pushes the channels
    // beyond the card's own as silence instead, and the status shows the
    // count that is real. A pretend "null:N" card has N.
    {
        int own = static_cast<int>(m_impl->device.playback.internalChannels);
        if (own <= 0) own = static_cast<int>(m_impl->device.playback.channels);
        if (nullOwn > 0) own = std::min(own, nullOwn);
        m_channelsActual.store(std::min(own, static_cast<int>(m_impl->device.playback.channels)), std::memory_order_release);
    }
    const int periodFrames = static_cast<int>(m_impl->device.playback.internalPeriodSizeInFrames);
    const int bufferFrames = periodFrames * static_cast<int>(m_impl->device.playback.internalPeriods);
    m_dsp.setPeriodFrames(periodFrames);
    // Prime silence so the servo starts at its target once the device has
    // filled its own buffer at the start.
    {
        const int prime = std::min(SHAKER_RING, m_dsp.target()
                        + static_cast<int>(std::ceil(bufferFrames * m_dsp.controlHz() / std::max(1, m_sampleRate))));
        double z[MAX_SHAKERS] = {};
        for (int i = 0; i < prime; ++i) m_dsp.push(z, MAX_SHAKERS);
    }
    m_bufferMs = 1000.0 * static_cast<double>(bufferFrames) / std::max(1, m_sampleRate);
    m_deviceName = m_impl->device.playback.name;
    // On ALSA, how the card is held (direct = every channel, low latency),
    // and the card at full line level.
    if (alsa && cfg.playback.pDeviceID)
    {
        m_deviceName += (cfg.playback.shareMode == ma_share_mode_exclusive) ? " [direct]" : " [shared]";
#if defined(__linux__)
        fullLineLevel(cfg.playback.pDeviceID->alsa);
#endif
    }
    // On Windows the same, and the card held at full volume (checkHealth
    // puts it back if anything turns it down); what it was is in the name,
    // so the start-up log says.
    if (wasapi)
    {
        m_deviceName += (cfg.playback.shareMode == ma_share_mode_exclusive) ? " [exclusive]" : " [shared]";
#if defined(_WIN32)
        float had = -1.0f; bool muted = false;
        if (holdFullVolume(m_impl->endpointId, had, muted))
        {
            char note[96];
            std::snprintf(note, sizeof note, " - the card was at %.0f %%%s: set to 100 %%",
                          std::max(0.0f, had) * 100.0f, muted ? " and muted" : "");
            m_deviceName += note;
        }
        m_volumeCheckMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
#endif
    }
    if (!fallbackReason.empty()) m_deviceName += " - " + fallbackReason;

    if (ma_device_start(&m_impl->device) != MA_SUCCESS)
    {
        ma_device_uninit(&m_impl->device); m_impl->deviceOk = false;
        m_error = "audio device would not start"; return false;
    }
    m_open.store(true, std::memory_order_release);
    return true;
}

void ShakerOutput::close()
{
    std::lock_guard<std::mutex> lk(m_mx);
    closeLocked();
}

void ShakerOutput::closeLocked()
{
    m_open.store(false, std::memory_order_release);   // the RT side's pushes skip the rings from here
    if (m_impl->deviceOk)
    {
        m_closing.store(true, std::memory_order_release);   // the stop we ask for is not a loss
        ma_device_uninit(&m_impl->device);   // stops the device and joins the audio thread
        m_closing.store(false, std::memory_order_release);
        m_impl->deviceOk = false;
    }
    m_channelsActual.store(0, std::memory_order_release);
}

bool ShakerOutput::reopen()
{
    std::string dev; int ch; double hz; int rate; bool asked;
    {
        std::lock_guard<std::mutex> lk(m_mx);
        dev = m_reqDevice; ch = m_reqChannels; hz = m_reqControlHz; rate = m_reqRate; asked = m_reqValid;
    }
    if (!asked) return false;   // never opened: nothing to open again
    return open(dev, ch, hz, rate);
}

void ShakerOutput::setControlHz(double hz)
{
    if (!isOpen()) return;
    m_dsp.configure(hz, m_sampleRate, m_dsp.channels());
}

} // namespace haptics
