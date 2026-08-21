// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// A6FaultCodes.h - StepperOnline A6-EC fault decoding.
//
// Source: A6-EC series servo drive user manual, Table 10-1 (ch 10.1.3).
// Two objects carry fault identity:
//   0x203F  panel code -- EXACT, one per Er code (0x871 = Er87.1). The panel
//           code follows the pattern Er<GG>.<S> -> 0x<GG><S>. On the wire
//           203F is a UInt32: LOW 16 bits = external (panel/Er) code -- the
//           value in the tables here -- high 16 bits = internal factory code.
//   0x603F  bus code   -- COARSE CiA402 class, many Er codes share one value
//           (0xFF00 alone covers Er11.0/45.0/84.3/87.1/87.2/87.3), which is
//           why a raw "code=0xff00" log line cost a whole investigation.
//
// Everything here is static const char* -- safe to pass to RT_LOG_* from the
// RT thread (no allocation, no formatting beyond the caller's printf).
//
// The Er03.x/Er05.x panel codes were resequenced from a column-misaligned
// PDF extraction using the ErGG.S -> 0xGGS pattern (validated on ~30
// unambiguous rows) and the independently known Er06.0 = bus 0x8400 anchor.
// ============================================================

#include <cstdint>

struct A6FaultInfo
{
    uint16_t    panel;        // 0x203F value
    const char* er;           // "Er87.1"
    const char* name;         // human-readable meaning
    bool        resettable;
    uint16_t    bus;          // 0x603F value this Er reports (0 = none listed)
};

inline const A6FaultInfo* a6FaultTable(int& count)
{
    static const A6FaultInfo t[] = {
        { 0x010, "Er01.0", "Mismatch of software versions",                false, 0x6100 },
        { 0x011, "Er01.1", "Mismatch of motor parameters",                 false, 0x7122 },
        { 0x020, "Er02.0", "Product matching fault: no specified drive",   false, 0x6100 },
        { 0x021, "Er02.1", "Product matching fault: no specified motor",   false, 0x6100 },
        { 0x022, "Er02.2", "Product matching fault: no specified encoder", false, 0x6100 },
        { 0x030, "Er03.0", "System parameter error",                       false, 0x6320 },
        { 0x031, "Er03.1", "Parameter out-of-range",                       false, 0x6320 },
        { 0x032, "Er03.2", "Parameter writing error",                      false, 0x6320 },
        { 0x033, "Er03.3", "Parameter reading error",                      false, 0x6320 },
        { 0x050, "Er05.0", "Current loop timeout",                         false, 0x7500 },
        { 0x051, "Er05.1", "Speed loop timeout",                           false, 0x7500 },
        { 0x052, "Er05.2", "Position loop timeout",                        false, 0x7500 },
        { 0x053, "Er05.3", "Serial port data check failure",               false, 0x7500 },
        { 0x060, "Er06.0", "Protection from out of control (runaway)",     false, 0x8400 },
        { 0x100, "Er10.0", "P-hardware overcurrent",                       false, 0x2312 },
        { 0x101, "Er10.1", "N-hardware overcurrent",                       false, 0x2312 },
        { 0x102, "Er10.2", "U phase software overcurrent",                 false, 0x2312 },
        { 0x103, "Er10.3", "V phase software overcurrent",                 false, 0x2312 },
        { 0x104, "Er10.4", "Output short circuited to ground",             false, 0x2330 },
        { 0x105, "Er10.5", "Current sampling failure",                     false, 0x6100 },
        { 0x106, "Er10.6", "Incorrect current parameter setting",          false, 0x6320 },
        { 0x107, "Er10.7", "UV current correction failure",                false, 0x6100 },
        { 0x108, "Er10.8", "Excessive current zero drift",                 false, 0x6100 },
        { 0x109, "Er10.9", "Current exception during enabling",            false, 0x2312 },
        { 0x110, "Er11.0", "Excessive motor speed at servo power-on",      false, 0xFF00 },
        { 0x111, "Er11.1", "Drive over-temperature",                       false, 0x2312 },
        { 0x201, "Er20.1", "Encoder internal fault",                       false, 0x7305 },
        { 0x202, "Er20.2", "Encoder reading/writing error",                false, 0x7305 },
        { 0x203, "Er20.3", "Encoder data frame loss",                      false, 0x7305 },
        { 0x204, "Er20.4", "Excessive encoder incremental position",       false, 0x7305 },
        { 0x205, "Er20.5", "Abnormal encoder data",                        false, 0x7305 },
        { 0x206, "Er20.6", "Mismatch of encoder type",                     false, 0x7305 },
        { 0x207, "Er20.7", "Encoder model not supported",                  false, 0x7305 },
        { 0x208, "Er20.8", "Encoder battery failure",                      false, 0x7305 },
        { 0x209, "Er20.9", "Encoder multi-turn error",                     false, 0x7305 },
        { 0x210, "Er21.0", "Encoder/drive pulses-per-rev mismatch",        false, 0x7305 },
        { 0x310, "Er31.0", "More than ten PDO mapping objects",            false, 0x8220 },
        { 0x320, "Er32.0", "EtherCAT peripheral error",                    false, 0x6100 },
        { 0x321, "Er32.1", "ESI check error in FLASH",                     false, 0x7600 },
        { 0x322, "Er32.2", "Failure to read EEPROM data through bus",      false, 0x7600 },
        { 0x323, "Er32.3", "Failure to update EEPROM through bus",         false, 0x7600 },
        { 0x324, "Er32.4", "ESC configuration area checksum error",        false, 0x7600 },
        { 0x325, "Er32.5", "EtherCAT failed to obtain valid XML info",     false, 0x7600 },
        { 0x400, "Er40.0", "Drive overload",                               true,  0x3230 },
        { 0x410, "Er41.0", "Motor overload",                               true,  0x3230 },
        { 0x411, "Er41.1", "Motor over-temperature (locked rotor)",        true,  0x7121 },
        { 0x412, "Er41.2", "Motor over-temperature",                       true,  0x4210 },
        { 0x421, "Er42.1", "Discharge tube temperature too high",          true,  0x4210 },
        { 0x422, "Er42.2", "Heatsink temperature too high",                true,  0x4210 },
        { 0x430, "Er43.0", "Overvoltage",                                  true,  0x3210 },
        { 0x431, "Er43.1", "Undervoltage",                                 true,  0x3220 },
        { 0x450, "Er45.0", "S-ON enabling failure",                        true,  0xFF00 },
        { 0x460, "Er46.0", "Motor overspeed",                              true,  0x8400 },
        { 0x470, "Er47.0", "Excessive position deviation",                 true,  0x8611 },
        { 0x471, "Er47.1", "Position deviation overflow",                  true,  0x8611 },
        { 0x501, "Er50.1", "D/Q current overflow",                         true,  0x6100 },
        { 0x510, "Er51.0", "Offline inertia auto-tuning failure",          true,  0x6310 },
        { 0x511, "Er51.1", "Offline inertia parameter error",              true,  0x6310 },
        { 0x520, "Er52.0", "Angle auto-tuning failure",                    true,  0x7122 },
        { 0x530, "Er53.0", "Motor parameter auto-tuning timeout",          true,  0x7122 },
        { 0x531, "Er53.1", "Resistance auto-tuning failure",               true,  0x7122 },
        { 0x532, "Er53.2", "Inductance auto-tuning failure",               true,  0x7122 },
        { 0x533, "Er53.3", "Back-EMF auto-tuning failure",                 true,  0x7122 },
        { 0x540, "Er54.0", "Current loop auto-tuning failure",             true,  0x7122 },
        { 0x550, "Er55.0", "Excessive vibration",                          true,  0x7122 },
        { 0x740, "Er74.0", "EtherCAT sync cycle setting error",            true,  0x6320 },
        { 0x741, "Er74.1", "No sync signal",                               true,  0x8700 },
        { 0x742, "Er74.2", "Chip sync process uncompleted in OP",          true,  0x8700 },
        { 0x800, "Er80.0", "Control power undervoltage",                   true,  0x3120 },
        { 0x810, "Er81.0", "Input phase loss 1",                           true,  0x3130 },
        { 0x811, "Er81.1", "Input phase loss 2",                           true,  0x3130 },
        { 0x812, "Er81.2", "Output phase loss (reserved)",                 true,  0x0000 },
        { 0x820, "Er82.0", "DI function allocation fault",                 true,  0x6320 },
        { 0x821, "Er82.1", "DO function allocation fault",                 true,  0x6320 },
        { 0x840, "Er84.0", "Electronic gear ratio setting error",          true,  0x6320 },
        { 0x841, "Er84.1", "Software limit setting error",                 true,  0x6320 },
        { 0x842, "Er84.2", "Encoder resolution setting error",             true,  0x7122 },
        { 0x843, "Er84.3", "Home position setting error",                  true,  0xFF00 },
        { 0x871, "Er87.1", "One-shot position increment > 5x max speed",   true,  0xFF00 },
        { 0x872, "Er87.2", "Position increment > max speed 3 cycles",      true,  0xFF00 },
        { 0x873, "Er87.3", "32-bit target position overflow",              true,  0xFF00 },
        { 0x874, "Er87.4", "Target position > max single-turn (rotating)", true,  0xFF00 },
        { 0xA01, "ErA0.1", "Multi-turn overflow fault",                    true,  0x7305 },
        // Class 2: EtherCAT communication faults (all resettable, all 0x8700)
        { 0xC10, "ErC1.0", "Excessive EtherCAT sync period error",         true,  0x8700 },
        { 0xC11, "ErC1.1", "Synchronization loss",                         true,  0x8700 },
        { 0xC12, "ErC1.2", "Network status switchover error",              true,  0x8700 },
        { 0xC14, "ErC1.4", "Network cable connection unreliable",          true,  0x8700 },
        { 0xC15, "ErC1.5", "Data frame loss protection error",             true,  0x8700 },
        { 0xC16, "ErC1.6", "Data frame forwarding error",                  true,  0x8700 },
        { 0xC17, "ErC1.7", "Data update timeout",                          true,  0x8700 },
        { 0xC18, "ErC1.8", "Watchdog expired",                             true,  0x8700 },
        { 0xC20, "ErC2.0", "SYNC signal loss",                             true,  0x8700 },
        // Class 3: factory ALARMS (warnings; don't trip the DS402 fault bit,
        // but 203F can report them). Panel codes pattern-derived ALFG.S ->
        // 0xFGS (unambiguous in Table 10-2); 603F left 0 where the extraction
        // was misaligned -- alarms never reach the 603F fault log path anyway.
        { 0xF00, "ALF0.0", "Emergency stop alarm",                         true,  0x0000 },
        { 0xF10, "ALF1.0", "Re-power-on required for parameter change",    true,  0x0000 },
        { 0xF11, "ALF1.1", "Frequent parameter storage alarm",             true,  0x0000 },
        { 0xF12, "ALF1.2", "Torque reached parameter error",               true,  0x0000 },
        { 0xF13, "ALF1.3", "Too frequent EEPROM writes by host SDO",       true,  0x0000 },
        { 0xF20, "ALF2.0", "Forward overtravel alarm",                     true,  0x0000 },
        { 0xF21, "ALF2.1", "Reverse overtravel alarm",                     true,  0x0000 },
        { 0xF40, "ALF4.0", "Homing timeout",                               true,  0x0000 },
        { 0xF41, "ALF4.1", "Homing DI conflict",                           true,  0x0000 },
        { 0xF42, "ALF4.2", "Homing mode conflict",                         true,  0x0000 },
        { 0xF50, "ALF5.0", "Braking resistor overload",                    true,  0x0000 },
        { 0xF51, "ALF5.1", "External regen resistance too small",          true,  0x0000 },
        { 0xF61, "ALF6.1", "Output phase loss",                            true,  0x0000 },
        { 0xF80, "ALF8.0", "Vibration during auto-tuning",                 true,  0x0000 },
        { 0xF90, "ALF9.0", "Encoder battery voltage low",                  true,  0x0000 },
        { 0xFA0, "ALFA.0", "Drive high temperature warning",               true,  0x0000 },
    };
    count = static_cast<int>(sizeof(t) / sizeof(t[0]));
    return t;
}

// Exact decode of the 0x203F panel code. nullptr if unknown.
inline const A6FaultInfo* a6PanelFault(uint16_t panel)
{
    int n = 0;
    const A6FaultInfo* t = a6FaultTable(n);
    for (int i = 0; i < n; ++i)
        if (t[i].panel == panel) return &t[i];
    return nullptr;
}

// Coarse decode of the 0x603F bus code: a static candidate string naming the
// Er codes that report this value, so a fault log line answers "which fault?"
// without a manual. Static literals only -- RT-safe to pass into RT_LOG_*.
inline const char* a6BusFaultCandidates(uint16_t bus)
{
    switch (bus)
    {
    case 0x2312: return "overcurrent/over-temp family (Er10.x/Er11.1)";
    case 0x2330: return "output short to ground (Er10.4)";
    case 0x3120: return "control power undervoltage (Er80.0)";
    case 0x3130: return "input phase loss (Er81.0/81.1)";
    case 0x3210: return "overvoltage (Er43.0)";
    case 0x3220: return "undervoltage (Er43.1)";
    case 0x3230: return "drive/motor overload (Er40.0/41.0)";
    case 0x4210: return "over-temperature (Er41.2/42.1/42.2)";
    case 0x6100: return "internal software fault (Er01/02/10.5/10.7/10.8/32.0/50.1)";
    case 0x6310: return "inertia auto-tune fault (Er51.x)";
    case 0x6320: return "parameter/setting error (Er03.x/10.6/74.0/82.x/84.0/84.1)";
    case 0x7121: return "motor locked rotor (Er41.1)";
    case 0x7122: return "motor/tuning parameter fault (Er01.1/52.0/53.x/54.0/55.0/84.2)";
    case 0x7305: return "encoder fault (Er20.x/21.0/A0.1)";
    case 0x7500: return "internal loop timeout (Er05.x)";
    case 0x7600: return "EEPROM/ESI storage fault (Er32.1-32.5)";
    case 0x8220: return "PDO mapping fault (Er31.0)";
    case 0x8400: return "runaway/overspeed protection (Er06.0/46.0)";
    case 0x8611: return "excessive position deviation (Er47.0/47.1)";
    case 0x8700: return "sync fault (Er74.1/74.2 drive-side, ErC1.0-C1.8 "
                        "EtherCAT comm, ErC2.0 SYNC loss)";
    case 0xFF00: return "manufacturer group: overspeed at power-on (Er11.0), "
                        "S-ON failure (Er45.0), home position error (Er84.3), "
                        "or EXCESSIVE POSITION INCREMENT (Er87.1-87.4) -- "
                        "panel/203F has the exact code";
    default:     return "unknown 603F code (see A6 manual Table 10-1)";
    }
}
