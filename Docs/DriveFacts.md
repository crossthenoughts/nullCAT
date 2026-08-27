# A6 Drive Facts (ANCTL AS715N)

## State Machine
| State | SDO | RPDO | TPDO | Notes |
|-------|-----|------|------|-------|
| Init (I) | No | No | No | Communication is initialized. There is no communication at the application layer, and the master can only read data from and write data to the ESC register. |
| IP | No | No | No |  The master configures the slave addresses. The mailbox channel is configured. The DC is configured. The pre-operational state is requested |
| Pre-Op (P) | Yes | No | No | Mailbox SDO communication at application layer |
| PS | Yes | No | No |  The master uses SDOs to initialize process data mapping. The master configures the SM channel used for process data communication. The master configures FMMU. The safe-operational state is requested.|
| Safe-Op (S) | Yes | No | Yes | Both SDOs and TPDOs available; DC mode available |
| SO | Yes | No | Yes | Master sends valid output data. Operational state is requested. |
| Operational (O) | Yes | Yes | Yes | Normal operation; all IO valid; mailbox available |

## PDO Mappings

PDO mapping is used to establish the mappings between the object dictionary and PDOs. 1600h to 17FFh
are RPDOs, and 1A00h to 1BFFh are TPDOs. The A6-EC series servo drive provides six RPDOs and five
TPDOs.

## Sync Modes
- The SYNC signal can be used for synchronization of all slaves and can achieve an error of less than
1 μs. Before the SYNC signal starts, the master needs to synchronize all slaves to the same clock.
In the operational state, it also needs to continuously synchronize the slaves to the same clock
to prevent the difference in the crystal oscillator from causing a clock offset. This is generally
manifested as synchronizing the 0x910 register of the ESC.

- The SYNC starting time is the time of the ESC 0x990 register minus the 0x920 time. Enable the DC
mode (0x981 = 0x03) before the 0x910 reaches the starting time. If the SYNC starting time setting
is incorrect, the ESC 0x134 status register will report a fault code of 0x2D.

- The A6-EC series servo drive only supports the DC sync mode. The synchronization cycle is
controlled by SYNC0. The period range varies with different motion modes.

### DC register width: 64-bit (confirmed)

The vendor manual, p156 (EtherCAT communication technical specifications table), states: **DC: 64 bits**.

SOEM's `ecx_dcsync0` writes 8 bytes (int64, little-endian via `htoell`) to register 0x0990–0x0997.
This is correct for this drive. Do not apply the SOEM Issue #364 Omron 1S workaround
(`t & 0xFFFFFFFF` to truncate to 32 bits): that fix targets drives with 32-bit DC only, and on the
A6-EC it would corrupt the start time by discarding the high 32 bits of a valid 64-bit value.

For reference, a representative startTime of 830919188750500000 ns (0x0B8804998A8780A0) writes:
- 0x0990–0x0993: `A0 80 87 8A` (lower 32 = 0x8A8780A0 = 2.324 s into 32-bit epoch)
- 0x0994–0x0997: `99 04 88 0B` (upper 32 = 0x0B880499)

The lower 32 bits alone would be 102 ms ahead of t1, a valid start time even in the 32-bit domain,
but the drive expects and uses the full 64-bit value.

### 0x09AE SYNC0 counter: reads but never counts (verified)

ESC register 0x09AE probes as readable on the A6-EC but never advances:
2708 field session, 2035 pre-OP pump samples across five init attempts,
all 0x0000 -- including on slaves that demonstrably pulsed (they accepted
OP and held DC sync for 29 minutes with zero sync errors). Do not use it
as a SYNC0 liveness signal. The working instrument is DCSTART0 (0x0990)
readback advance: a live pulse unit advances it to the next upcoming
pulse (small positive margin vs DCSYSTIME); a wedged one leaves the
original programmed start frozen in the past.

### Sync quality (verified)

ESC register 0x092C (DC System Time Difference) measures the offset between the slave's local clock and the master DC reference time. Encoding is sign-magnitude: bit 31 is a direction flag (1 = slave clock is behind master, 0 = slave ahead), bits 30:0 are the magnitude in nanoseconds. [Beckhoff ESC datasheet]

ArthurKetels' canonical threshold for a correctly synchronised system is <1 µs. Measured on this hardware: across all three A6-EC slaves, sysdiff converges to <100 ns by approximately cycle 1000 of the pre-OP pump (~2 seconds at 1ms cadence) and remains there. DC clock drift is not a candidate cause of ErC1.1 in this system.

### Sync error mechanism (Sync Mode 2)

With C13.05=2 (mandatory on Windows+Npcap hosts, whose frame-timing jitter exceeds the 1 µs that mode 1 tolerates), the A6-EC counts each SYNC0 window as a sync error if the number of PDO frames received in that window is ≠ 1. Examples:

- 1ms pump at 500Hz (2ms SYNC0 period): 2 frames per window, so every window is a sync error.
- 2ms pump at 500Hz: 1 frame per window, no errors.
- 0 frames in a window: also a sync error (e.g. any PDO gap longer than 2ms).

The sync error counter saturates after enough events and fires ErC1.1. A pump rate that disagrees with the SYNC0 period is therefore a structural cause of ErC1.1, confirmed on this hardware.

### DC start time activation

`ecx_dcsync0()` hardcodes `SyncDelay = 100,000,000 ns = 100ms` (SOEM `ec_dc.c`). The first SYNC0 pulse is always scheduled at least 100ms after the call, specifically at the next whole multiple of CyclTime that is ≥ t1 + SyncDelay. [SOEM source]

**The `marginNs` diagnostic value is not the activation-time margin.** It is a post-hoc snapshot taken after `ecx_config_map_group()` finishes (~56ms for three drives) and after the register-readback loop. It measures how much of the original 100ms headroom remains at the time of measurement. For slave 1 (whose hook fires first inside configMap), measured `marginNs` ranged from 25 µs to 1.2 ms; almost the full 100ms is consumed by configMap execution. This is expected and does not indicate a timing problem: SYNC0 was scheduled 100ms in the future at call time, and the measurement is simply taken close to that scheduled time. Slaves 2 and 3 show more remaining margin because their hooks fire later within configMap.

## Drive Parameters
### Group C13 (manual §11.3.7)

C13.00: EtherCAT slave name
- Indicates the station number assigned to the slave by the master during EtherCAT communication.

C13.01: EtherCAT slave alias
- Indicates the station number assigned to the slave EtherCAT communication since the master cannot automatically assign station numbers.
- C13.01 = 0: The master assigns the station numbers by default. C13.01 ≠ 0: The set station number applies by default, with the one assigned by master deactivated.

C13.05: EtherCAT synchronization mode setting
- Defines the synchronization work mode:
Setpoint 
0= manufacturer
1= Applies to the scenarios where the synchronization performance 
indicator of the host controller jitters for 1 us.
2= Applies to the scenarios where the synchronization performance 
indicator of the host controller jitters for more than 1 us.
- In the work mode, the synchronization cycle must be an integer multiple of 125 μs. Otherwise, the serve 
drive will report Er74.0 (EtherCAT synchronization cycle setting is incorrect.)

C13.06: EtherCAT synchronization error threshold
- Defines the permissible jitter range of synchronization signals when the servo drive works in synchronization 
mode 1 (C13.05 = 1).

C13.08: EtherCAT enhanced link selection
- When a redundant loop network is used, the EtherCAT Enhanced Link Check function must be enabled 
(C13.08 = 1), which will take effect upon next power-on of the servo drive.
- When a loop network is used, both C13.08 and C13.19 need to be set to 1

## Fault Codes
### Er74.0 
(EtherCAT synchronization cycle setting is incorrect.)
### Er74.1
Er74.1 No sync signal
Cause: The communication synchronization clock is configured incorrectly for the master.
Solution: Correct the master communication configuration.
### ErC1.1
ErC1.1 Synchronization loss.
Causes:
- The communication synchronization clock is configured incorrectly for the master.
- The IN and OUT ports for EtherCAT communication are connected inversely.
- The controller chip of the slave is damaged.
- The MCU pin is damaged.
Solutions:
- Perform the test on another master. Correct the master communication configuration.
- Connect the IN and OUT ports in the correct sequence.
- If the fault persists after the master is replaced, use an oscilloscope to measure the synchronization signal generated by the slave controller chip. If there is no signal, the slave controller chip is damaged. Return to 
factory for repair or replace the controller chip of the slave.
- Use an oscilloscope to measure the 
synchronization signal generated by the 
slave controller chip. If there is signal, the MCU chip pin is damaged. Return to factory for repair or replace the MCU chip.
### ErC1.2
ErC1.2 Network status switchover error.
Cause: Master malfunction or manual malfunction occurs.
Solution: Check the network status switchover program of the host controller.
- Use a shielded twisted-pair communication cable.
- Ground the servo drive according to the 
standard.
- Check the network connection status 
according to the LED.
### ErC1.8
ErC1.8 Watchdog expired.
Cause: The master configuration is 
incorrect.
Solution: Modify the watchdog configuration of the host controller
### ErC2.0
ErC2.0 SYNC signal loss. 
Cause: The physical connection of the data link is unstable, or the process data is lost due to network cable connection 
and removal.
 Solution: Replace the network cable with a more reliable one. If the fault persists, contact technical support.

## DS402 Statusword Notes

### Manufacturer-specific bit 8

During sync error recovery the A6-EC sets bit 8 of the DS402 statusword. Bit 8 is manufacturer-specific territory in DS402 (the standard warning bit is bit 7) and is not part of any standard state-word bit pattern. The drive can send `SW = 0x0101` (bits 0 and 8 set).

Standard DS402 parsing does not match `0x0101` to any defined state and returns Unknown. Any master state machine that waits for a defined DS402 state must expect this pattern during sync-error recovery; left unhandled, it stalled the homing state machine on this rig while waiting for a state transition that never arrived.

## Mailbox Protocol

The A6-EC reports `mbx_proto = 0x000C`:
- Bit 2 (0x04): CoE (CAN application protocol over EtherCAT) is supported
- Bit 3 (0x08): FoE (File access over EtherCAT) is supported
- Bit 1 (0x02): EoE (Ethernet over EtherCAT) is **not supported**

SOEM Issue #217 describes mailbox frame interference when a slave supports both EoE and CoE simultaneously. This issue does not apply to the A6-EC. Verified on hardware: a diagnostic mailbox drain read SM1 and confirmed mbx_proto=0x000C with zero emergency messages.

## SOEM Notes

Distilled from the SOEM maintainer's forum answers.

### Configuration and PDO mapping

SOEM is a library with many functions; the stock configuration and mapping functions are one way of doing it. If you have other requirements, write other functions to do the job.

SOEM handles dynamic PDO mapping. Its configuration logic first reads the slave ID via the SII (eeprom), then reads the capabilities from SII to see whether CoE or SoE are supported. If they are, the mapping follows the CoE/SoE structure; if not (or if CoE/SoE mapping fails), it falls back to the mapping in SII.

SOEM normally assigns slave addresses starting from 0x1001. If a slave first appears at a different address and is later reassigned to 0x1001, recovery code that searches for the old address will not find it.

If a slave defines no mapping (SDO 0x1a00:01 and 0x1600:01 read back 0x00000000), CoE mapping fails and SOEM falls back to SII (eeprom) for the PDO configuration.

### Concurrent PDO and SDO traffic

SOEM supports running PDO in one thread and SDO in another without either blocking the other, subject to scheduler performance. It works best with a real-time scheduler, with the PDO thread at higher priority than the SDO thread.

### Mailbox emergency messages and SDO crashes

Mailbox emergency messages interfere with normal SDO functions. After the slave reaches pre-OP, poll its out mailbox for some time to drain any emergency messages, then start the first SDO transfers. Always check the result of any SDO function with `ecx_iserror` for abnormal results (including emergency messages).

Drain recipe:
1. `ecx_getmbx()` fetches a mailbox buffer from the pool; check for NULL.
2. `ecx_mbxreceive()` tries to read the mailbox from the slave.
3. `ecx_dropmbx()` returns the used buffer.

Drain time depends on the slave, anywhere from milliseconds to seconds. Ideally configure the slave to not send emergency packets at all; they are a leftover from CANopen. A NULL dereference inside `ecx_mbxreceive` typically means SOEM ran out of mailbox stack.

### SDO writes before ecx_config_map_group

Assure in the application that the slave is at least in pre-OP before commencing mailbox traffic. Check (and optionally set) slave state before configuring slave PDOs and after; only then do mapping. You never know when a slave resets or misbehaves.

`ecx_readstate()` is very efficient (only one BRD if all slaves have the same state) and updates the slave state for all entries in the slavelist. It is often overlooked in application code.

The PO2SO hook is the preferred way to configure slaves, per the SOEM documentation.

## RT-path Logging

All log call sites on the RT thread (`ControlLoopWorker::run()` and every per-cycle function it calls) use the `RT_LOG_*` macros, never `LOG_*(strf(...))`. `LOG_*` takes `const std::string&`, so every `strf()` call on the RT path would construct a heap-allocated `std::string` before entering the logger. `RT_LOG_*` expands to `Logger::pushRT()`, which formats directly into the pre-allocated 479-byte `LogEntry.msg` buffer via `vsnprintf`: no heap allocation, no mutex.

The bad-frame warning in `ControlLoop.cpp` is rate-limited. A WKC mismatch during a cascade would otherwise fire every cycle; it is guarded by `badFrameCount % 100 == 1` (fires at 1, 101, 201, ...) with the counter reset to 0 on the first clean frame.

## Known Limitation: Background Pump Cadence During Long OP Holds

**Symptom:** Drives show SYNC0 watchdog faults (Er74.1 / ErC1.1) if the app is
left in OP for more than about 8s before the control loop is started. Observed on
hardware: gaps of 9s and 11s faulted; gaps of 5–6s were clean.

**Root cause (hypothesis):** The background pump thread (`m_pumpThread`, 2ms
cadence) runs with `THREAD_PRIORITY_TIME_CRITICAL`, but it is still subject to
OS scheduler jitter. During long holds the cumulative phase drift between pump
frames and the drive's SYNC0 window exceeds the drive's watchdog tolerance.
Drives are more sensitive to this on slave 1 (first in chain).

**Mitigated** by removing the enforced UI wait between OP and starting the
control loop. With the wait removed, the gap is typically under 500ms (user
reaction time), which is below the watchdog threshold observed in testing.

**Not fully solved:** The background pump cadence issue remains latent. Long
intentional holds at OP (pre-flight checks, manual intervention) could still
trigger the fault. A proper fix would require DC-locked pump timing (aligning
the 2ms pump deadline to DCSYSTIME) or a shorter pump period with watchdog
heartbeat SDOs.

## Drive Enable Transition Fault Mode

### Root cause

When the DS402 state machine transitions from SwitchedOn (controlword 0x07) to
OperationEnabled (controlword 0x0F), the drive snapshots its internal command-position
register. If the command register differs from the encoder reading by more than the
0x6065 following-error window (100mm), the drive immediately faults with ErP1.8 or
similar.

This does not happen during steady-state CSP operation because target PDO updates
are continuous. It happens exclusively at the SwitchedOn to OperationEnabled transition,
because the iomap is zero-initialized before the pump starts, so the command register
starts at 0 (or wherever it was at the previous power cycle) while the encoder may be
at any position.

### Fix

`A6Drive::stepEnableStateMachine()` holds the controlword at 0x07 for
`m_commandSyncCycles` cycles (default 10, configurable via `commandSyncCycles` in
host.json) while writing `*m_pTargetPosition = m_lastActualCounts` every cycle.
This forces the drive's command register to converge to the current encoder position
before the 0x0F snapshot.

On the Nth cycle, 0x0F is written. The drive now snapshots a command position that
matches the encoder, so no following error can occur at enable time.

### HomingSequence interaction

HomingSequence enters SettlingDrive immediately after drive enable. If the drive faults
during settle, the settle-specific fault message is logged:

```
HomingSequence[N]: Drive faulted during settle (SW=0x...) -- pre-enable sync was not
sufficient. Aborting homing.
```

This message distinguishes a sync failure (fault within SETTLE_CYCLES cycles of enable)
from a fault during active torque search.

### Configuration

`commandSyncCycles` (default 10) can be lowered if testing shows the drive's command
register converges faster. Do not set below 3: the drive needs at least 2-3 PDO
exchanges to latch the new command position before 0x0F is sent.

---

## Known Limitations and Open Questions

Items surfaced during hardware log review.

### 1. Drive ESC opacity: no public register map

The A6-EC manual does not publish an ESC register map. ALStatusCode, ALControl, and
DC registers are readable via SOEM (ecx_FPRD/ecx_FPWR), but their meaning must be
inferred from the EtherCAT spec and empirical observation. Fault diagnosis beyond
ALState/ALCode is not possible without the register map or a Wireshark trace.

### 2. SDO crash sequence not covered by safeCall

`ecx_SDOwrite` inside the PO2SO hook is called from within `ecx_config_map_group()`,
which is already inside `safeCall`. A second SEH crash originating inside the hook
is caught by the outer safeCall, but the `failedSlave` field is populated from the
outer context (group index), not the slave that caused the crash. Diagnosis of
SDO-level failures inside the hook requires Wireshark.

### 3. Re-arm path SEH gap

`stageDCArm()` re-arms SYNC0 via `ecx_dcsync0()` when `marginNs` is low. This call
is not wrapped in `safeCall`. A crash here would propagate as an unhandled exception
past `tryInitOnce()`. Low risk (re-arm is unconditional), but not zero.

### 4. sendReceive SEH cascade not rate-limited

If `safeSendReceive()` catches a crash, it logs `LOG_ERROR` and sets `m_pumpCrashed`.
If the crash is transient (e.g. NIC DMA stall), subsequent calls continue. But if
the NIC is permanently down, every call fires an SEH catch. The crash log is unbounded.
Consider a one-shot flag or a rate limit, as already done for WKC mismatches.

### 5. Background pump cadence not DC-locked

The 2ms pump thread uses `PlatformRT::waitUntil()` (QPC-based). The drive's SYNC0
window is DC-locked. On a loaded Windows system, accumulated QPC drift can cause
multiple or zero frames in a SYNC0 window, triggering ErC1.1. The correct fix is to
align the pump deadline to DCSYSTIME. Deferred: the mitigation above (short gap
between OP and control-loop start) keeps the fault rate low in practice.

### 6. PDO watchdog readback not verified

The PDO watchdog is configured by writing the ESC watchdog registers (0x0400 divider,
0x0410 SM watchdog time) via FPWR. The write return codes are checked, but the
registers are not read back. If a slave silently ignores the write, there is no
indication. Add a readback of 0x0400/0x0410 after the write.

### 7. Drive 0 (slave 1) fragility profile

Slave 1 is consistently the first to report ErC1.1 in long-hold tests and the first
to fail during pump cadence jitter. This is expected: slave 1 is the chain root and
sees raw frame-delivery variance before EtherCAT framing smooths it for slaves 2+.
Any future latency analysis should treat slave 1 as the sentinel.

### 8. Negative marginNs observation

`stageDCArm()` has observed `marginNs < 0` (SYNC0 start time already past by the
time of measurement). This triggers the re-arm path. The re-arm appears to succeed,
but it is unclear whether the drive's DC clock handles a backwards-adjustment
gracefully. No fault has been observed, but this needs a hardware trace with a
known-negative margin to confirm.

### 9. Abnormally fast SAFE-OP precursor

In one hardware run, the drive reached SAFE-OP in <50ms (vs. typical 200–400ms).
This was followed by a stagePreOpPump timeout. Hypothesis: the drive reset
mid-init and re-entered its fast boot path, landing in SAFE-OP before the
master's timing assumptions held. The `discoverAndPrepareSlaves()` mailbox ping
may need an additional post-SAFE-OP settle delay for drives that boot unusually fast.
