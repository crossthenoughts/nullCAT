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
#include <cstdlib>

#include <cstring>

namespace haptics {

struct ShakerOutput::Impl
{
    ma_context context;
    bool       contextOk = false;
    ma_device  device;
    bool       deviceOk = false;
};

ShakerOutput::ShakerOutput() : m_impl(new Impl)
{
    ma_backend backends[] = { ma_backend_wasapi, ma_backend_alsa, ma_backend_null };
    ma_context_config cfg = ma_context_config_init();
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
    close();
    std::lock_guard<std::mutex> lk(m_mx);
    m_error.clear();
    std::string fallbackReason;   // why a shared device was opened instead of the raw one (empty = it was not)
    if (!m_impl->contextOk) { m_error = "no audio backend available"; return false; }
    channels = std::max(1, std::min(MAX_SHAKERS, channels));

    ma_device_config cfg = ma_device_config_init(ma_device_type_playback);
    cfg.playback.format   = ma_format_f32;
    cfg.playback.channels = static_cast<ma_uint32>(channels);
    cfg.sampleRate        = static_cast<ma_uint32>(sampleRate);
    cfg.dataCallback      = reinterpret_cast<ma_device_data_proc>(&ShakerOutput::dataCallback);
    cfg.pUserData         = this;
    cfg.periodSizeInFrames = 240;    // 5 ms at 48 kHz
    cfg.periods            = 3;

    // Device by name (substring, case-insensitive), "" = default, "null" = the null backend.
    // On ALSA the card itself is miniaudio's ":card,dev" id (or "hw:..."),
    // listed beside shared names with the same description (default:,
    // sysdefault:, front:, ...): prefer the card itself, then plughw:, then
    // whatever else matched.
    ma_device_id chosen; bool haveId = false;
    const bool wantNull = (deviceName == "null");
    const bool alsa = (m_impl->context.backend == ma_backend_alsa);
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
    // On ALSA ask for the card exclusively: in shared mode miniaudio opens
    // its "dmix" layer first (two channels, a 16384-frame buffer: eight
    // shakers squeezed into stereo and 60 ms or more late; found on the
    // bench), exclusively the card itself ("hw": every channel, the periods
    // asked for). A card another program holds falls back to shared below.
    if (alsa && !wantNull) cfg.playback.shareMode = ma_share_mode_exclusive;

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
            fallbackReason = std::string("card not available on its own (") + ma_result_description(r)
                           + "), shared: 2 channels, high latency";
            cfg.playback.shareMode = ma_share_mode_shared;
            r = ma_device_init(&m_impl->context, &cfg, &m_impl->device);
        }
    }
    if (r != MA_SUCCESS) { m_error = std::string("audio device open failed: ") + ma_result_description(r); return false; }
    m_impl->deviceOk = true;

    m_sampleRate = static_cast<int>(m_impl->device.sampleRate);
    m_dsp.configure(controlHz, m_sampleRate, static_cast<int>(m_impl->device.playback.channels));
    // Prime half a ring of silence so the servo starts centred.
    { double z[MAX_SHAKERS] = {}; for (int i = 0; i < SHAKER_RING / 2; ++i) m_dsp.push(z, MAX_SHAKERS); }
    m_bufferMs = 1000.0 * static_cast<double>(m_impl->device.playback.internalPeriodSizeInFrames)
               * static_cast<double>(m_impl->device.playback.internalPeriods) / std::max(1, m_sampleRate);
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
    m_open.store(false, std::memory_order_release);
    if (m_impl->deviceOk)
    {
        ma_device_uninit(&m_impl->device);   // stops the device and joins the audio thread
        m_impl->deviceOk = false;
    }
}

void ShakerOutput::setControlHz(double hz)
{
    if (!isOpen()) return;
    m_dsp.configure(hz, m_sampleRate, m_dsp.channels());
}

} // namespace haptics
