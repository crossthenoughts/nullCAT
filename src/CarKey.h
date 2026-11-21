// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// CarKey - the car a sender names (its game, its name, its id), as one
// number: what is remembered per car between drives goes by it.
//
// Not the same question as otherCar() (DeviceStateLayer.h), which says when
// the laws start their per-car learning over: there an id arriving under a
// name already known is the same car, and the game is not looked at. Here
// the three strings are the key as they stand, so the same name in another
// game is another car, and so is a name before and after its id arrives (a
// sender on record sends the two on one line).
//
// Pure arithmetic over at most three kCarKeyStrLen strings: RT-safe.
// ============================================================

#include <cstdint>

static constexpr int kCarKeyStrLen = 48;   // each of the three strings (NCY_STR_LEN)

// FNV-1a over "game|car|id". Never 0 (0 = no car named).
inline uint64_t carIdentityKey(const char* game, const char* car, const char* carId)
{
    uint64_t h = 1469598103934665603ull;
    const auto eat = [&h](const char* s)
    {
        for (int i = 0; i < kCarKeyStrLen && s[i] != '\0'; ++i) { h ^= static_cast<unsigned char>(s[i]); h *= 1099511628211ull; }
        h ^= static_cast<unsigned char>('|'); h *= 1099511628211ull;
    };
    eat(game); eat(car); eat(carId);
    return h != 0 ? h : 1;
}
