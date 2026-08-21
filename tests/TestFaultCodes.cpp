// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestFaultCodes.cpp - unit tests for the A6 fault decode tables.
//
// Proves (a) the panel-code lookup resolves the codes we have actually seen
// in the field, (b) every table row's panel code follows the ErGG.S -> 0xGGS
// pattern the resequencing relied on, (c) panel codes are unique, and (d) the
// 603F candidate strings mention every Er code that maps to that bus value.
// No SOEM / Qt / web / NIC. Run with: ctest -R FaultCodes
// ============================================================
#include "A6FaultCodes.h"
#include <cstdio>
#include <cstring>
#include <string>

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("  FAIL: %s  (line %d)\n", (msg), __LINE__); } \
} while (0)

int main()
{
    // ---- (a) Field-relevant decodes ----
    // ErC1.1 -- the historic sync-loss nemesis
    {
        const A6FaultInfo* f = a6PanelFault(0xC11);
        CHECK(f && std::strcmp(f->er, "ErC1.1") == 0, "0xC11 -> ErC1.1");
        CHECK(f && f->bus == 0x8700, "ErC1.1 bus code is 0x8700");
        CHECK(f && f->resettable, "ErC1.1 is resettable");
    }
    // Er87.1 -- drive 5's suspected 0xFF00 (catch-up burst increment)
    {
        const A6FaultInfo* f = a6PanelFault(0x871);
        CHECK(f && std::strcmp(f->er, "Er87.1") == 0, "0x871 -> Er87.1");
        CHECK(f && f->bus == 0xFF00, "Er87.1 bus code is 0xFF00");
    }
    // Er47.0 following error, Er43.1 undervolt, Er06.0 runaway anchor
    {
        const A6FaultInfo* f = a6PanelFault(0x470);
        CHECK(f && f->bus == 0x8611, "Er47.0 bus code is 0x8611");
        f = a6PanelFault(0x431);
        CHECK(f && f->bus == 0x3220, "Er43.1 bus code is 0x3220");
        f = a6PanelFault(0x060);
        CHECK(f && std::strcmp(f->er, "Er06.0") == 0 && f->bus == 0x8400,
              "Er06.0 anchor: 0x060 -> bus 0x8400");
    }
    // Unknown code -> nullptr, not a bogus row
    CHECK(a6PanelFault(0x999) == nullptr, "unknown panel code -> nullptr");
    CHECK(a6PanelFault(0x0000) == nullptr, "zero panel code -> nullptr");

    // ---- (b) Er/ALF name <-> panel code pattern holds for every row ----
    // ErGG.S -> 0xGGS (hex digits taken verbatim from the name). This is the
    // invariant the misaligned-extraction resequencing relied on; if a row
    // breaks it, the row was transcribed wrong.
    {
        int n = 0;
        const A6FaultInfo* t = a6FaultTable(n);
        CHECK(n > 80, "table has full coverage (>80 rows)");
        for (int i = 0; i < n; ++i)
        {
            const char* e = t[i].er;   // "Er87.1" or "ALF4.0"
            size_t len = std::strlen(e);
            bool alarm = (std::strncmp(e, "ALF", 3) == 0);
            const char* digits = alarm ? e + 3 : e + 2;   // "87.1" / "4.0"
            // Build expected code from the digit chars around the dot.
            std::string hexStr;
            for (const char* p = digits; *p; ++p)
                if (*p != '.') hexStr += *p;
            if (alarm) hexStr = "F" + hexStr;             // ALF4.0 -> F40
            unsigned expected = std::stoul(hexStr, nullptr, 16);
            char msg[96];
            std::snprintf(msg, sizeof(msg), "%s panel code 0x%03X matches pattern 0x%03X",
                          e, t[i].panel, expected);
            CHECK(t[i].panel == expected, msg);
            (void)len;
        }

        // ---- (c) Panel codes unique ----
        for (int i = 0; i < n; ++i)
            for (int j = i + 1; j < n; ++j)
            {
                char msg[64];
                std::snprintf(msg, sizeof(msg), "duplicate panel code 0x%03X", t[i].panel);
                CHECK(t[i].panel != t[j].panel, msg);
            }

        // ---- (d) Candidate strings name every mapped Er family ----
        // For each fault row with a bus code, the candidate string for that
        // bus code must mention the Er group (e.g. "Er87" covers Er87.1-4;
        // "Er20.x" covers all Er20 subs). Alarm rows (bus 0) are exempt.
        for (int i = 0; i < n; ++i)
        {
            if (t[i].bus == 0) continue;
            std::string cand = a6BusFaultCandidates(t[i].bus);
            // Group token: "Er87", "ErC1", "Er06" -> also accept bare group
            // digits ("47.0/47.1" style) since some strings elide the Er prefix.
            std::string group(t[i].er, 4);               // "Er87" / "ErC1"
            std::string bare = group.substr(2);          // "87" / "C1"
            bool mentioned = cand.find(group) != std::string::npos
                          || cand.find(bare)  != std::string::npos;
            char msg[128];
            std::snprintf(msg, sizeof(msg), "candidates for 0x%04X mention %s: \"%.60s\"",
                          t[i].bus, t[i].er, cand.c_str());
            CHECK(mentioned, msg);
        }
    }

    // ---- 603F fallbacks ----
    CHECK(std::strstr(a6BusFaultCandidates(0x1234), "unknown") != nullptr,
          "unknown 603F -> explicit unknown string");
    CHECK(std::strstr(a6BusFaultCandidates(0xFF00), "203F") != nullptr,
          "0xFF00 string points at the panel code");

    std::printf("TestFaultCodes: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
