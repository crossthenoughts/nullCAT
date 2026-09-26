// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// libFuzzer target for the wire-facing telemetry line parser.
//
// parsePacket runs on the RT thread against raw UDP datagrams from the
// network, so any crash, overflow, or sanitizer finding here is a
// remote-input bug by definition. The parser is a pure static function
// (no socket, no state), which makes this target exact: bytes in,
// nothing else involved.
//
// Built and run by CI only (clang, -fsanitize=fuzzer,address) - see the
// `fuzz` job in .github/workflows/ci.yml. Seeds: tests/fuzz/corpus/.
// ============================================================
#include "TelemetryInput.h"
#include <cstdint>
#include <cstddef>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    if (size > 2048) return 0;   // real datagrams are short single lines
    TelemetryData td;
    TelemetryInput::parsePacket(reinterpret_cast<const char*>(data),
                                static_cast<int>(size), td);
    return 0;
}
