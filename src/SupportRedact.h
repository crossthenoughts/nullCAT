// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// The web password (host.json "webAuthToken") blanked in a settings file's
// text before it goes into a support bundle: a bundle is made to be shared.
// The same rule as deploy/support-bundle.ps1 and support-bundle.sh:
// escape-aware (a value with \" in it is one value), an empty password
// stays empty (no password set), and anything left that is not blanked (a
// value that is not a string, one that never ends) makes it return false,
// so the caller leaves the file out rather than risk it.
// ============================================================

#include <cctype>
#include <string>

inline bool redactWebAuthToken(std::string& text)
{
    static const std::string kKey   = "\"webAuthToken\"";
    static const std::string kBlank = "\"<redacted>\"";
    size_t from = 0;
    for (;;)
    {
        const size_t k = text.find(kKey, from);
        if (k == std::string::npos) return true;
        size_t i = k + kKey.size();
        while (i < text.size() && std::isspace(static_cast<unsigned char>(text[i]))) ++i;
        if (i >= text.size() || text[i] != ':') { from = k + kKey.size(); continue; }   // the name as a value, not a key
        ++i;
        while (i < text.size() && std::isspace(static_cast<unsigned char>(text[i]))) ++i;
        if (i >= text.size() || text[i] != '"') return false;   // not a string
        size_t j = i + 1;
        while (j < text.size() && text[j] != '"') j += (text[j] == '\\') ? 2 : 1;
        if (j >= text.size()) return false;                     // never ends
        if (j > i + 1) text.replace(i, j - i + 1, kBlank);
        from = i + 1;
    }
}
