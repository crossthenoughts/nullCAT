// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// EffectStatus - sticky per-sim, per-effect status (effectstatus.json
// beside host.json/rig.json). NOT config: remembered machine state, like
// CarCache, never merged or validated.
//
// For every game the channel stream has named (NULLCATY game=, or "" for
// a sender that does not say) and every effect in the registry, it
// remembers whether the channels the effect needs have EVER been
// delivered by that game, whether the effect has ever actually produced
// output, and when it was last seen. The web paints a dot per tile from
// it: green = has produced, amber = channels arrive but nothing was
// ever felt (amplitude 0, no route, or a law that never triggers), grey
// = that game never delivered the channels. Persisting it answers "does
// this sim feed this effect at all?" without a drive in the pits.
//
// observe() is called from a sampling thread (the web server's), never
// the RT loop; it reads the controller's published status snapshot.
// ============================================================

#include "HapticsRegistry.h"
#include "NcxTokens.h"
#include <array>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

struct EffectRecord
{
    bool    delivered  = false;   // the effect's required channels have arrived from this game
    bool    produced   = false;   // the effect has put out something while that game ran
    int64_t lastSeenMs = 0;       // wall clock, ms since epoch, last time delivered or produced
};

class EffectStatus
{
public:
    using Records = std::array<EffectRecord, haptics::EFFECT_COUNT>;

    bool load(const std::string& anchorPath);
    bool save(const std::string& anchorPath) const;

    // One status sample. game: the stream's identity ("" = unnamed sender).
    // have: which tokens the stream delivers right now; fxOut: the layer's
    // output level per FxType slot (0 when nothing can be felt); firedBy:
    // transient counters per EventType; streamLive: channel stream fresh.
    void observe(const std::string& game, const bool* have, const double* fxOut,
                 const uint64_t* firedBy, bool streamLive, int64_t nowMs);

    // Does the registry's channel spec read as delivered from these tokens?
    // The same syntax the web uses: "a|b" any-of, "group*" all four wheel
    // tokens, "~x" optional (ignored here). No channels = delivered.
    static bool channelsDelivered(const haptics::EffectInfo& info, const bool* have);

    std::string              currentGame() const;
    std::vector<std::string> games() const;          // most recently seen first
    Records                  records(const std::string& game) const;
    void clear(const std::string& game);             // forget one game's records
    void clearAll();
    bool dirty() const { std::lock_guard<std::mutex> lk(m_mx); return m_dirty; }

private:
    struct Game { Records rec; int64_t lastSeenMs = 0; };
    std::map<std::string, Game> m_games;
    std::string m_current;
    uint64_t    m_lastFiredBy[haptics::EVENT_TYPE_COUNT] = {};
    bool        m_haveFired = false;
    mutable std::mutex m_mx;
    mutable bool m_dirty = false;
};
