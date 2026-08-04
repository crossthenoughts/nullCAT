// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestStatusModel.cpp — unit tests for the shared status model.
//
// Proves the model (a) reproduces the current web derivation, (b) retains raw
// state for future un-folding, (c) has a deterministic single colour table, and
// (d) computes the aggregate by the defined precedence. No SOEM / Qt / web / NIC.
// Run with: ctest -R StatusModel   (or run the binary directly).
// ============================================================
#include "StatusModel.h"
#include <cstdio>
#include <cctype>
#include <string>

using namespace status;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("  FAIL: %s  (line %d)\n", (msg), __LINE__); } \
} while (0)

static const char* nm(Indicator s) { return styleOf(s).name; }

// ---- Golden reference: an independent C++ port of the CURRENT web app.js status
// derivation (ds402() + stateClass() + the card merge). The model must reproduce
// this for every realistic input, or the refactor would change the web. ----
static const char* DASH = "\xE2\x80\x94";  // em dash, matches app.js '—'
static std::string lc_(std::string s){ for(char& c : s) c=(char)std::tolower((unsigned char)c); return s; }
static bool has_(const std::string& h, const char* n){ return h.find(n) != std::string::npos; }

struct RefElec { std::string t; std::string sc; bool fault; };
static RefElec ref_ds402(uint16_t sw){
    uint16_t s = sw;
    if((s&0x4F)==0x08 || (s&0x4F)==0x0F) return {"FAULT","s-fault",true};
    if((s&0x6F)==0x07) return {"QUICK STOP","s-fault",true};
    if((s&0x6F)==0x27) return {"OP ENABLED","s-op",false};
    if((s&0x6F)==0x23) return {"SWITCHED ON","s-home",false};
    if((s&0x6F)==0x21) return {"READY","s-park",false};
    if((s&0x4F)==0x40) return {"SW-ON DISABLED","s-off",false};
    if((s&0x4F)==0x00) return {"NOT READY","s-off",false};
    char b[8]; std::snprintf(b,sizeof(b),"0x%04X",sw); return {std::string(b),"s-off",false};
}
static std::string ref_stateClass(const std::string& st){
    std::string s = lc_(st);
    if(has_(s,"fault")||has_(s,"fatal")) return "s-fault";
    if(has_(s,"estop")) return "s-estop";
    if(has_(s,"hom")) return "s-home";
    if(has_(s,"op")||has_(s,"online")||has_(s,"run")) return "s-op";
    if(has_(s,"park")) return "s-park";
    return "s-off";
}
// app.js card derivation. hasSw = ('sw' in d); motion = d.state (present only when loop).
static void ref_card(bool hasSw, uint16_t sw, const std::string& motion, bool loop,
                     std::string& text, std::string& cls){
    RefElec elec; bool elecP = hasSw; if(hasSw) elec = ref_ds402(sw);
    std::string ml = lc_(motion);
    bool motionValid = loop && !motion.empty() &&
        (has_(ml,"park")||has_(ml,"hom")||has_(ml,"oper")||has_(ml,"run")||
         has_(ml,"fault")||has_(ml,"estop")||has_(ml,"fatal"));
    if(elecP && elec.fault){ text=elec.t; cls="s-fault"; return; }
    if(motionValid){ text=motion; cls=ref_stateClass(motion); return; }
    if(elecP){ text=elec.t; cls=elec.sc; return; }
    text = motion.empty() ? std::string(DASH) : motion; cls = ref_stateClass(motion);
}

// helper: derive with common defaults (rawDrive unused by derivation, kept as detail)
static AxisIndicator drv(int i, AxisMotionState m, const std::string& name,
                         uint16_t sw, bool loop, bool estop = false, bool hasSw = true)
{
    return deriveAxis(i, m, name, hasSw, sw, DriveState::Unknown, loop, estop);
}

int main()
{
    // ---- Acceptance #1: axis1 fault, axes 0&2 OP-enabled, loop running, no estop ----
    {
        AxisIndicator a[3] = {
            drv(0, AxisMotionState::ONLINE, "ONLINE", 0x0027, true),   // OP ENABLED
            drv(1, AxisMotionState::ONLINE, "ONLINE", 0x0008, true),   // FAULT
            drv(2, AxisMotionState::ONLINE, "ONLINE", 0x0027, true),   // OP ENABLED
        };
        CHECK(a[0].state == Indicator::RUNNING, "acc#1 axis0 RUNNING");
        CHECK(a[1].state == Indicator::FAULT,   "acc#1 axis1 FAULT");
        CHECK(a[1].fault,                        "acc#1 axis1 fault flag");
        CHECK(a[2].state == Indicator::RUNNING, "acc#1 axis2 RUNNING");
        Indicator agg = deriveAggregate(a, 3, /*loop*/true, /*estop*/false);
        CHECK(agg == Indicator::FAULT, "acc#1 aggregate FAULT");
    }

    // ---- Current foldings preserved + RAW state retained for un-folding ----
    {
        AxisIndicator on = drv(0, AxisMotionState::ONLINE,   "ONLINE",   0x0027, true);
        CHECK(on.state == Indicator::RUNNING,                 "ONLINE -> RUNNING (electrical)");
        CHECK(on.text  == "OP ENABLED",                       "ONLINE text = electrical OP ENABLED");
        CHECK(on.rawMotion == AxisMotionState::ONLINE,        "ONLINE raw retained");

        AxisIndicator bl = drv(0, AxisMotionState::BLENDING, "BLENDING", 0x0027, true);
        CHECK(bl.state == Indicator::RUNNING,                 "BLENDING -> RUNNING (folded)");
        CHECK(bl.rawMotion == AxisMotionState::BLENDING,      "BLENDING raw retained for un-fold");

        AxisIndicator up = drv(0, AxisMotionState::UNPARKING, "UNPARKING", 0x0027, true);
        CHECK(up.state == Indicator::IDLE,                    "UNPARKING -> IDLE (folded)");
        CHECK(up.text  == "UNPARKING",                        "UNPARKING text = motion name");
        CHECK(up.rawMotion == AxisMotionState::UNPARKING,     "UNPARKING raw retained for un-fold");

        AxisIndicator hm = drv(0, AxisMotionState::HOMING, "HOMING[search]", 0x0027, true);
        CHECK(hm.state == Indicator::BUSY,                    "HOMING -> BUSY");
        CHECK(hm.text  == "HOMING[search]",                   "HOMING sub-state text preserved");
    }

    // ---- electrical fault wins over motion; loop-not-running uses electrical ----
    {
        AxisIndicator f = drv(0, AxisMotionState::HOMING, "HOMING[search]", 0x0008, true);
        CHECK(f.state == Indicator::FAULT, "electrical fault wins over HOMING");

        AxisIndicator p = drv(0, AxisMotionState::PARKED, "PARKED", 0x0040, /*loop*/false);
        CHECK(p.state == Indicator::OFFLINE && p.text == "SW-ON DISABLED",
              "loop stopped -> electrical (SW-ON DISABLED) not motion");
    }

    // ---- DS402 electrical decode coverage (matches web ds402) ----
    {
        struct { uint16_t sw; Indicator st; const char* tx; } cases[] = {
            { 0x0027, Indicator::RUNNING, "OP ENABLED"     },
            { 0x0023, Indicator::BUSY,    "SWITCHED ON"    },
            { 0x0021, Indicator::IDLE,    "READY"          },
            { 0x0040, Indicator::OFFLINE, "SW-ON DISABLED" },
            { 0x0000, Indicator::OFFLINE, "NOT READY"      },
            { 0x0008, Indicator::FAULT,   "FAULT"          },
            { 0x0007, Indicator::FAULT,   "QUICK STOP"     },
        };
        for (auto& c : cases)
        {
            // ONLINE+loop is non-meaningful, so derivation uses the electrical branch
            AxisIndicator r = drv(0, AxisMotionState::ONLINE, "ONLINE", c.sw, true);
            CHECK(r.state == c.st, "elec decode state");
            CHECK(r.text == c.tx,  "elec decode text");
        }
        AxisIndicator hx = drv(0, AxisMotionState::ONLINE, "ONLINE", 0x1234, true);
        CHECK(hx.state == Indicator::OFFLINE && hx.text == "0x1234", "unknown sw -> hex text, OFFLINE");
    }

    // ---- aggregate precedence: ESTOP > FAULT > RUNNING > OFFLINE ----
    {
        AxisIndicator ok[2] = {
            drv(0, AxisMotionState::ONLINE, "ONLINE", 0x0027, true),
            drv(1, AxisMotionState::ONLINE, "ONLINE", 0x0027, true),
        };
        CHECK(deriveAggregate(ok, 2, true,  false) == Indicator::RUNNING, "agg RUNNING when running+clean");
        CHECK(deriveAggregate(ok, 2, false, false) == Indicator::OFFLINE, "agg OFFLINE when stopped+clean");
        CHECK(deriveAggregate(ok, 2, true,  true ) == Indicator::ESTOP,   "agg ESTOP overrides all");
        AxisIndicator fault[2] = {
            drv(0, AxisMotionState::ONLINE, "ONLINE", 0x0008, true),  // fault
            drv(1, AxisMotionState::ONLINE, "ONLINE", 0x0027, true),
        };
        CHECK(deriveAggregate(fault, 2, true, false) == Indicator::FAULT, "agg FAULT when any axis faults");
        CHECK(deriveAggregate(fault, 2, true, true ) == Indicator::ESTOP, "agg ESTOP beats FAULT");
    }

    // ---- single deterministic colour/blink table (renderers cannot diverge) ----
    {
        CHECK(std::string(styleOf(Indicator::RUNNING).webClass) == "s-op",    "RUNNING -> s-op");
        CHECK(std::string(styleOf(Indicator::BUSY   ).webClass) == "s-home",  "BUSY -> s-home");
        CHECK(std::string(styleOf(Indicator::IDLE   ).webClass) == "s-park",  "IDLE -> s-park");
        CHECK(std::string(styleOf(Indicator::OFFLINE).webClass) == "s-off",   "OFFLINE -> s-off");
        CHECK(std::string(styleOf(Indicator::FAULT  ).webClass) == "s-fault", "FAULT -> s-fault");
        CHECK(std::string(styleOf(Indicator::ESTOP  ).webClass) == "s-estop", "ESTOP -> s-estop");
        CHECK(std::string(styleOf(Indicator::FAULT  ).pattern)  == "blink",   "FAULT pattern blink");
        CHECK(std::string(styleOf(Indicator::BUSY   ).pattern)  == "pulse",   "BUSY pattern pulse");
        // determinism: same reference returned each call
        CHECK(&styleOf(Indicator::RUNNING) == &styleOf(Indicator::RUNNING),   "styleOf deterministic ref");
    }

    // ---- Golden-reference equivalence sweep: model == app.js across the matrix ----
    // Proves the refactor changes nothing on the web (acceptance #2), automatically,
    // instead of relying on manual console-watching during shakedown.
    {
        struct M { AxisMotionState st; const char* name; };
        const M motions[] = {
            {AxisMotionState::HOMING,"HOMING[search]"}, {AxisMotionState::PARKED,"PARKED"},
            {AxisMotionState::PARKING,"PARKING"},       {AxisMotionState::UNPARKING,"UNPARKING"},
            {AxisMotionState::BLENDING,"BLENDING"},     {AxisMotionState::ONLINE,"ONLINE"},
            {AxisMotionState::ESTOPPING,"E-STOP"},
        };
        const uint16_t sws[] = {0x0008,0x000F,0x0007,0x0027,0x0023,0x0021,0x0040,0x0000,0x1234,0x0637};
        struct OP { bool masterOp, loop; };
        const OP ops[] = {{true,true},{true,false},{false,false}};  // loop implies OP -> skip (F,T)
        int sweepN = 0, sweepDiff = 0;
        for (const auto& mo : motions)
        for (uint16_t sw : sws)
        for (const auto& op : ops)
        {
            // emit pipeline: d.state present only while the loop runs; statusword only when OP
            std::string webMotion = op.loop ? std::string(mo.name) : std::string();
            std::string oT, oC;
            ref_card(op.masterOp, sw, webMotion, op.loop, oT, oC);

            AxisMotionState mst = op.loop ? mo.st : AxisMotionState::PARKED;
            AxisIndicator nInd = deriveAxis(0, mst, webMotion, op.masterOp, sw,
                                            DriveState::Unknown, op.loop, false);
            std::string nT = nInd.text.empty() ? std::string(DASH) : nInd.text;
            std::string nC = styleOf(nInd.state).webClass;

            ++sweepN;
            if (nT != oT || nC != oC){
                ++sweepDiff; ++g_fail;
                std::printf("  SWEEP DIFF: motion=%s sw=0x%04X op=%d loop=%d | old=[%s,%s] new=[%s,%s]\n",
                    mo.name, sw, op.masterOp?1:0, op.loop?1:0,
                    oT.c_str(), oC.c_str(), nT.c_str(), nC.c_str());
            } else ++g_pass;
        }
        std::printf("  equivalence sweep: %d combos, %d card diffs vs app.js reference\n", sweepN, sweepDiff);
    }

    // ---- Documented intentional divergence: aggregate logo vs old app.js ----
    {
        // Old app.js logo: driveFault = any d.state matches /fault|fatal/. d.state is a
        // motion name (HOMING/PARKED/.../E-STOP) -> never contains fault/fatal, so the
        // old logo's is-fault branch is DEAD. The model aggregate correctly reports
        // FAULT from the statusword -> intentional improvement (the one logo diff).
        const char* motionNames[] = {"HOMING[search]","PARKED","PARKING","UNPARKING","BLENDING","ONLINE","E-STOP"};
        bool oldLogoEverFault = false;
        for (auto n : motionNames){ std::string s = lc_(n); if(has_(s,"fault")||has_(s,"fatal")) oldLogoEverFault = true; }
        CHECK(!oldLogoEverFault, "old logo driveFault is dead code (motion names never say fault)");
        AxisIndicator fa[1] = { drv(0, AxisMotionState::ONLINE, "ONLINE", 0x0008, true) };
        CHECK(deriveAggregate(fa,1,true,false) == Indicator::FAULT,
              "model aggregate reports FAULT where old logo never did (intentional)");
    }

    // ================================================================
    // V1 review S2: rig-level boolean aggregates — the SINGLE SOURCE for
    // /api/status parked/homing/hasBelts/beltsSlack AND the toggle
    // endpoints' resolution. Rules copied literally from the three former
    // hand-derivation sites; these pins are what make the consolidation
    // safe to rely on.
    // ================================================================
    {
        using MS = AxisMotionState;

        // ---- motion aggregates ----
        {   // empty rig: allParked seeded false (numDrives==0), nothing set
            MotionAggregates a = deriveMotionAggregates(nullptr, 0);
            CHECK(!a.allParked && !a.anyHoming && !a.transitional,
                  "agg motion: empty rig -> all false");
        }
        {   MS st[3] = { MS::PARKED, MS::PARKED, MS::PARKED };
            MotionAggregates a = deriveMotionAggregates(st, 3);
            CHECK(a.allParked && !a.anyHoming && !a.transitional,
                  "agg motion: all PARKED -> allParked, settled");
        }
        {   MS st[3] = { MS::PARKED, MS::ONLINE, MS::PARKED };
            MotionAggregates a = deriveMotionAggregates(st, 3);
            CHECK(!a.allParked && !a.transitional,
                  "agg motion: ONLINE -> not parked, NOT transitional");
        }
        {   MS st[2] = { MS::PARKED, MS::PARKING };
            CHECK(deriveMotionAggregates(st, 2).transitional,
                  "agg motion: PARKING is transitional");
        }
        {   MS st[2] = { MS::PARKED, MS::UNPARKING };
            CHECK(deriveMotionAggregates(st, 2).transitional,
                  "agg motion: UNPARKING is transitional");
        }
        {   MS st[2] = { MS::PARKED, MS::HOMING };
            MotionAggregates a = deriveMotionAggregates(st, 2);
            CHECK(a.transitional && a.anyHoming,
                  "agg motion: HOMING is transitional AND sets anyHoming");
        }
        {   MS st[2] = { MS::PARKED, MS::ESTOPPING };
            CHECK(deriveMotionAggregates(st, 2).transitional,
                  "agg motion: ESTOPPING is transitional");
        }
        {   // Pinned legacy: BLENDING is NOT motion-transitional (park-toggle
            // during a blend resolves "park"; startPark handles BLENDING axes).
            MS st[2] = { MS::PARKED, MS::BLENDING };
            MotionAggregates a = deriveMotionAggregates(st, 2);
            CHECK(!a.transitional && !a.allParked,
                  "agg motion: BLENDING NOT transitional (pinned legacy)");
        }

        // ---- belt aggregates ----
        {   bool tq[2] = { false, false };
            MS st[2] = { MS::PARKED, MS::PARKED };
            BeltAggregates b = deriveBeltAggregates(st, 2, tq, 2);
            CHECK(!b.hasBelts && !b.beltsSlack,
                  "agg belt: no torque axes -> hasBelts false, slack false");
        }
        {   bool tq[2] = { false, true };
            MS st[2] = { MS::ONLINE, MS::PARKED };
            BeltAggregates b = deriveBeltAggregates(st, 2, tq, 2);
            CHECK(b.hasBelts && b.beltsSlack && !b.transitional,
                  "agg belt: torque PARKED -> slack (non-torque state irrelevant)");
        }
        {   bool tq[2] = { false, true };
            MS st[2] = { MS::PARKED, MS::ONLINE };
            BeltAggregates b = deriveBeltAggregates(st, 2, tq, 2);
            CHECK(b.hasBelts && !b.beltsSlack && !b.transitional,
                  "agg belt: torque ONLINE -> tensioned, NOT transitional");
        }
        {   bool tq[1] = { true };
            for (MS t : { MS::PARKING, MS::BLENDING, MS::UNPARKING, MS::ESTOPPING })
            {
                MS st[1] = { t };
                CHECK(deriveBeltAggregates(st, 1, tq, 1).transitional,
                      "agg belt: torque transitional state refuses toggle");
            }
        }
        {   // non-torque axis transitioning must NOT set belt transitional
            bool tq[2] = { false, true };
            MS st[2] = { MS::PARKING, MS::PARKED };
            BeltAggregates b = deriveBeltAggregates(st, 2, tq, 2);
            CHECK(!b.transitional && b.beltsSlack,
                  "agg belt: non-torque PARKING ignored by belt aggregates");
        }
        {   // pinned quirk: torque axis beyond the engine snapshot counts slack
            bool tq[2] = { false, true };
            MS st[1] = { MS::PARKED };
            BeltAggregates b = deriveBeltAggregates(st, 1, tq, 2);
            CHECK(b.hasBelts && b.beltsSlack,
                  "agg belt: torque axis beyond numStates counts as slack (quirk)");
        }
    }

    std::printf("%d/%d checks passed.\n", g_pass, g_pass + g_fail);
    return g_fail == 0 ? 0 : 1;
}
