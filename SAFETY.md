# ⚠️ Safety

**Read this before powering anything.**

nullCAT commands industrial servo drives that move a motion rig, typically
a seat with a person in it. These drives can produce forces **capable of
causing serious injury or death**: servo actuators under a fast control
loop can crush fingers, tip a rig, or snap a belt tensioner without
warning. Never rely on software alone for safety. Treat this like
industrial machine commissioning, because it is.

## The rules

1. **A hardware emergency stop is REQUIRED.** A latched, mechanical e-stop
   that removes drive power **even when this software has crashed, hung,
   or powered off** (in practice: interrupting a contactor that feeds the
   drives; see the wiring note below). The e-stop button in the web UI and the optional GPIO
   panel stop are **SOFTWARE stops**. They ask a running program to
   disable the drives, so they are convenience controls, not safety
   devices. If the software is dead, only the hardware latch stops the
   rig.
2. **Physical end-stops on every axis.** Software limits, homing offsets,
   and stroke guards are best-effort; the mechanics must survive a
   full-speed runaway into the stop.
3. **Nobody on the rig during commissioning.** First power-on, homing
   validation, tuning, firmware or config changes: chair empty, hands
   clear, e-stop within reach.
4. **Belt tensioners (CST torque mode) deserve respect.** A torque-mode
   drive pulls until told otherwise. Verify the drive-side torque limit,
   the tension limits, and the e-stop to zero-torque path before wearing
   the belts.
5. **Validate every change on the rig before trusting it.** This project's
   own development rule: one change at a time, verified on hardware.
   Config edits count as changes.
6. **Follow your drive manufacturer's safety instructions** and your local
   electrical and machinery regulations. Mains-powered servo drives demand
   proper earthing, fusing, and enclosure practice.

## About wiring the e-stop (informal guidance, not instruction)

The A6-EC drives used by the reference rig have no STO (Safe Torque Off)
input, so there is no drive terminal that gives you a certified safe-stop
path. Two things follow from that:

- The robust approach is a latching mushroom that interrupts the coil of a
  contactor feeding the drives' motor power. Power that is physically
  removed cannot be overridden by any software or drive state. This is
  what the reference rig uses.
- Wiring a stop to a drive digital input is a drive-firmware function, not
  an STO. It can be a useful convenience layer, but it should never be
  your only stop.

Sizing and wiring a mains contactor is electrician territory and depends
on your drives, supply, and local regulations. Treat this section as
orientation, not instruction: the drive manual and a qualified electrician
are the authorities here.

## No warranty

This software is distributed under the GPL **WITHOUT ANY WARRANTY**; without
even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE (see Sections 15 and 16 of LICENSE). It is not certified for
safety-critical use, carries no safety rating of any kind, and has not been
assessed against any machinery, functional-safety, or electrical standard.

You are the machine builder. Under most machinery regulations, the person
who assembles components into a working rig and puts it into service is
responsible for the safety of the resulting machine. Compliance with local
machinery regulations, electrical safety, and safe installation are yours.
**By installing, commissioning, or using this software you assume all risk
of doing so, including the risk of property damage, personal injury, or
death.**
