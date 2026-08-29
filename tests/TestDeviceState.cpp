// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestDeviceState.cpp - unit tests for the NULLCATX state layer.
//
// Pins the binding resolution (NcxMap: token/slot/scale/offset, bad
// bindings skipped, missing slots unbound) and the DeviceStateLayer
// effect logic (clutch blocking + gear grind), with the fail-safe rules
// front and centre: stale stream, unbound token, or inert config all
// yield inert mods. Plain-main, no Qt, runs on every platform.
// ============================================================

#include "../src/DeviceStateLayer.h"
#include <cstdio>
#include <cmath>

static int g_pass = 0, g_fail = 0;
static void CHECK(bool ok, const char* what)
{
    if (ok) { ++g_pass; }
    else    { ++g_fail; std::printf("FAIL: %s\n", what); }
}

static TelemetryData ncxFrame(std::initializer_list<double> ch, bool fresh = true)
{
    TelemetryData td{};
    td.numNcx = 0;
    for (double v : ch) td.ncx[td.numNcx++] = v;
    td.ncxFresh = fresh;
    return td;
}

int main()
{
    // ================= NcxMap binding resolution =================
    {
        std::vector<NcxBinding> bs = {
            { "rpm",       0, 1.0,   0.0 },
            { "clutchPct", 3, 100.0, 0.0 },   // wire 0..1 -> percent
            { "speedKmh",  1, 3.6,   0.0 },   // wire m/s -> km/h
            { "nonsense",  2, 1.0,   0.0 },   // unknown token: skipped
            { "gear",      99, 1.0,  0.0 },   // slot out of range: skipped
        };
        NcxMap map; map.configure(bs);
        CHECK(map.boundCount() == 3, "bad bindings are skipped at configure");

        const TelemetryData td = ncxFrame({ 6500.0, 40.0, 7.0, 0.35 });
        NcxValues v = map.extract(td);
        CHECK(v.fresh, "fresh mirrors the snapshot");
        CHECK(v.have[NcxValues::Rpm] && std::fabs(v.val[NcxValues::Rpm] - 6500.0) < 1e-12,
              "rpm passes through at scale 1");
        CHECK(v.have[NcxValues::ClutchPct] &&
              std::fabs(v.val[NcxValues::ClutchPct] - 35.0) < 1e-12,
              "scale maps wire units onto the token convention");
        CHECK(v.have[NcxValues::SpeedKmh] &&
              std::fabs(v.val[NcxValues::SpeedKmh] - 144.0) < 1e-12,
              "m/s -> km/h via scale");
        CHECK(!v.have[NcxValues::Gear] && !v.have[NcxValues::ThrottlePct],
              "unbound tokens report have=false");

        const TelemetryData shortTd = ncxFrame({ 6500.0 });   // 1 channel only
        v = map.extract(shortTd);
        CHECK(v.have[NcxValues::Rpm] && !v.have[NcxValues::ClutchPct],
              "a slot beyond the packet's channel count is unbound this cycle");
    }

    // ================= DeviceStateLayer effects =================
    DeviceParams p;
    p.detents       = { -0.05, 0.0, 0.05 };
    p.clutchBitePct = 25.0;
    p.blockGain     = 1.5;
    p.grindAmpPct   = 12.0;
    p.grindFreqHz   = 47.0;
    p.blockStartRev = 0.01;

    std::vector<NcxBinding> bs = { { "clutchPct", 0, 1.0, 0.0 } };
    NcxMap map; map.configure(bs);
    DeviceStateLayer layer; layer.configure(p);

    const auto inert = [](const DeviceStateMods& m)
    { return m.forceScale == 1.0 && m.neutralShift == 0.0 &&
             m.textureAmpPct == 0.0 && m.textureFreqHz == 0.0; };

    {
        // Blocked push: clutch up (5% < bite 25%), lever 0.02 rev out of
        // the 0.0 detent (> blockStart 0.01).
        NcxValues v = map.extract(ncxFrame({ 5.0 }));
        DeviceStateMods m = layer.step(0.02, v);
        CHECK(std::fabs(m.forceScale - 2.5) < 1e-12, "blocked shift: forceScale = 1 + blockGain");
        CHECK(m.textureAmpPct == 12.0 && m.textureFreqHz == 47.0,
              "blocked shift: grind texture on");

        m = layer.step(0.02, map.extract(ncxFrame({ 80.0 })));
        CHECK(inert(m), "clutch pressed past the bite: no block, no grind");

        m = layer.step(0.005, map.extract(ncxFrame({ 5.0 })));
        CHECK(inert(m), "inside blockStartRev of the detent: no effect");

        // The NEAREST detent governs: at 0.055 the lever is only 0.005 from
        // the 0.05 gate (settled in gear -> inert); at 0.03 it is 0.02 from
        // that same gate (mid-shift between gates -> blocked).
        m = layer.step(0.055, map.extract(ncxFrame({ 5.0 })));
        CHECK(inert(m), "settled near a non-neutral gate: no effect");
        m = layer.step(0.03, map.extract(ncxFrame({ 5.0 })));
        CHECK(!inert(m), "mid-shift between gates with clutch up: blocked");
    }

    // ---- fail-safe rules ----
    {
        NcxValues stale = map.extract(ncxFrame({ 5.0 }, /*fresh=*/false));
        CHECK(inert(layer.step(0.02, stale)), "stale stream: inert (plain feel)");

        NcxMap unbound; unbound.configure({});
        CHECK(inert(layer.step(0.02, unbound.extract(ncxFrame({ 5.0 })))),
              "clutch token not bound: inert");

        DeviceParams off = p; off.clutchBitePct = 0.0;
        DeviceStateLayer loff; loff.configure(off);
        CHECK(inert(loff.step(0.02, map.extract(ncxFrame({ 5.0 })))),
              "clutchBitePct 0 disables the whole clutch logic");

        DeviceParams noDet = p; noDet.detents.clear();
        DeviceStateLayer lnd; lnd.configure(noDet);
        CHECK(inert(lnd.step(0.02, map.extract(ncxFrame({ 5.0 })))),
              "no detents (pedal): clutch logic never applies");
    }

    // ---- composition: the mods actually shape the force model ----
    {
        DeviceParams fp;
        fp.springCurve = { {0.0, 0.0}, {0.07, 100.0} };
        fp.velLpfHz = 0.0; fp.dampPctPerRevS = 0.0;
        fp.maxForcePct = 300.0; fp.slewPctPerSec = 1e9;
        DeviceForceModel plain; plain.configure(fp, 0.0005); plain.reset(0.0);
        DeviceForceModel blocked; blocked.configure(fp, 0.0005); blocked.reset(0.0);
        DeviceStateMods none;
        DeviceStateMods block; block.forceScale = 2.5;
        const double f0 = plain.step(0.02, none);
        const double f1 = blocked.step(0.02, block);
        CHECK(std::fabs(f1 - 2.5 * f0) < 1e-9,
              "block forceScale multiplies the field through the model");
    }

    std::printf("TestDeviceState: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
