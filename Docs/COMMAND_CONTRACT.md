# nullCAT Command Contract

This is the binding contract for every control surface: web UI, desktop UI,
GPIO panel, button-box bindings. Surfaces are thin triggers; all meaning
lives here. The contract tests (TestCommandContract, TestHttpContract)
assert this table. Changing behavior means changing the table and the tests
in the same commit.

## Architecture facts every consumer must know

1. **Two-layer guards.** The HTTP layer checks only component readiness and
   returns fast. Motion commands are ENQUEUED, and the engine applies the
   real preconditions on the RT thread. Consequence: `{"ok":true}` means
   **accepted**, not **done**. An engine-level refusal (for example
   TensionBelts under e-stop) is silent on the wire.
2. **Therefore: outcome truth comes from `/api/status` polling, never from
   the command response.** Binder LED feedback and UI state must key off
   status fields (loopRunning, masterOp, estop, parked, beltsSlack, homing,
   guard states), not off POST results.
3. **Toggles are SERVER-resolved, guarded, and first-class.** One physical
   button per stateful pair binds a `-toggle` token; the server resolves it
   against canonical engine state, so button and rig cannot drift apart.
   Two mandatory guards apply: **transitions are no-ops, never reversals**
   (a toggle acts only from a settled state), and a **1.5s per-toggle
   cooldown** swallows double-press and bounce flip-flops. The response
   carries `"resolved":"<action>"`. Discrete half-endpoints remain for the
   dashboard, scripts, and legacy bindings.
4. All commands are idempotent: re-sending in the already-achieved state is
   a no-op (or a fast `ok`), never an error and never a reversal.

## Toggle table

| Toggle | Resolves to | From | Guards |
|---|---|---|---|
| `init-toggle`  | `deinit` if OP else `init` | `masterOp` | cooldown; init/deinit busy-refusal inherited |
| `run-toggle`   | `stop` if running else `start` | `loopRunning` | cooldown; start still requires OP |
| `park-toggle`  | `unpark` if all parked else `park` | axis states | cooldown; NO-OP while any axis PARKING/UNPARKING/HOMING/ESTOPPING |
| `belts-toggle` | `belts/tension` if slack else `belts/slack` | belt axis states | cooldown; NO-OP while belt transitioning; tension-under-e-stop refused VISIBLY here (`ok:false`) |

## Command table

| Command | Endpoint | HTTP-layer guard (errResp) | Engine-layer guard (silent; visible via status/log) | Outcome signal in /api/status | Bindable |
|---|---|---|---|---|---|
| Initialize | `/api/init` | refused if already operational / init busy / not ready | - (async bring-up; failure surfaces in status + log) | `masterOp:true`, `slavesFound` | yes |
| De-initialize | `/api/deinit` | refused if not initialized / busy | seat pass runs bus-health guard (may SKIP seat, still de-inits) | `masterOp:false` | yes |
| Start loop | `/api/start` | needs master operational; ok if already running | - | `loopRunning:true` | yes |
| Stop loop | `/api/stop` | none (no-op when stopped) | parks axes as part of stop | `loopRunning:false`, `parked:true` | yes |
| Home | `/api/home` (optional body `{"axis":N}`) | needs motion controller; out-of-range `axis` refused | ignored per-axis unless axis eligible (belt axes never home) | `homing:true` → `needsRehome:false` | yes |
| Park | `/api/park` | needs motion controller | per-axis state machine | `parked:true` | yes |
| Unpark | `/api/unpark` | needs motion controller | **position axes only**: belts NEVER tension via unpark, auto or explicit; requires homed axes | `parked:false` | yes |
| Slack belts | `/api/belts/slack` | needs motion controller | torque axes only; allowed in any non-fault state | `beltsSlack:true` | yes |
| Tension belts | `/api/belts/tension` | needs motion controller | **REFUSED under e-stop** (logged, silent on wire); torque axes only; from PARKED/PARKING | `beltsSlack:false` | yes |
| Software e-stop | `/api/estop` | needs components | - (always honored) | `estop:true` | yes |
| E-stop release | `/api/estop/release` | none | - | `estop:false` | **desktop-only** (re-arm needs eyes on the rig) |
| Reset fault (+lockout) | `/api/reset-fault` | needs components | drive may refuse reset until thermal decay (Er40/41); retried by fault monitor | drive state via per-drive status; lockout cleared | yes |
| Reset stats | `/api/resetstats` | needs loop | - | tuning metrics re-baseline | no (tuning, not operation) |
| Restart service | `/api/restart` | private client address only | - | - | **never bindable** |
| Power off | `/api/shutdown` | private client address only | - | - | **never bindable** |

Notes:
- `/api/home` accepts an optional JSON body `{"axis": N}` (1-based, chain
  order) to home a single axis; no body, an unparseable body, or `axis: 0`
  homes all axes. Out-of-range `axis` is refused at the HTTP layer. Built
  for hexapod bring-up: per-leg direction checks with the pushrods
  disconnected, and near-park single-leg re-homes. A full solo sweep on a
  coupled platform binds the mechanism - see HEXAPOD_SETUP.md's homing
  section. The bindable `home` token remains home-all.
- `clear-lockout` is deliberately NOT a separate endpoint: `/api/reset-fault`
  is the one-button recovery (drive fault reset THEN software lockout clear).
- `/api/estop` is convenience on top of the HARDWARE latch (the safety
  device); nothing in this contract is a safety path.
- Config endpoints (`/api/rig`, `/api/host`) are not commands and are
  outside this table (fields documented in CONFIG_REFERENCE.md).
- The bindable-command set is enforced server-side when saving button
  bindings: a crafted POST cannot bind restart, shutdown, or e-stop release.

## Request-level security guards

These run in the web server before any command logic:

- **Host-header allowlist.** Every request is checked before routing. A
  missing or foreign `Host` header gets `421 Misdirected Request` and no
  handler runs. This is the DNS-rebinding defense: a rebinding page carries
  its own hostname in `Host`, which never matches this machine. Accepted
  names: the machine's own interface addresses, localhost forms, the
  hostname (and `<hostname>.local`), plus any names listed in
  `webAllowedHosts` in host.json.
- **Private-client gate on power commands.** `/api/shutdown` and
  `/api/restart` additionally require a private (loopback or LAN) client
  address. A non-private source gets 403 even with an allowed Host header.
- **No CORS headers anywhere.** The UI is served same-origin; cross-origin
  pages get nothing back.

Pinned by TestHttpContract.

## Belt-tension rule

Belt tension has exactly ONE trigger: TensionBelts (`/api/belts/tension`,
or `belts-toggle` resolving to it). The post-homing auto-unpark excludes
torque axes, so neither e-stop release nor `/api/home` can tension a belt
as a side effect, and unpark cannot bypass the overspeed guard's latched
slack. Vertical axes keep full auto-recovery (release, re-home, unpark).
TensionBelts is also the only latch-clearing re-tension; it remains refused
under e-stop. Pinned by TestCommandContract and the TestTorquePath
guard-latch tests.

## Known contract gaps

- G1: engine-level refusals return `ok:true` (accepted-not-done).
  Acceptable under fact #2; revisit only if a surface genuinely needs
  synchronous refusal codes.
- G2: `/api/home` and `/api/unpark` engine preconditions (homed/eligible)
  are enforced per-axis in the state machine; the contract tests must pin
  the observable behavior (command in wrong state = state unchanged).
- G3: SIMULATION cannot complete a torque-hardstop homing search (no sim
  torque model; homing targets bypass sim actuals), so a vertical axis
  started homing in sim sits in HOMING forever. TestHttpContract therefore
  runs a belt-only rig. A sim homing/hardstop model would lift this and
  enable sim-mode demos of the full lifecycle.
