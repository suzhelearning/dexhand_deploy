# Bounded timestamp-aware proposal continuity

User authorized fixing the observed `proposal exceeds maximum command step`
fault. Sequential TDD, no commits/agents/live-session restart or hardware writes.

Design: v131-only coordinator config opts into a 50 ms source-time window.
Allowed displacement is max(two nominal steps, nominal joint speed times elapsed
proposal time capped at the window), with a 1e-10 rad floating tolerance.
Reference is the timestamp of the proposal actually used for the last final
command, not the most recently received proposal, callback time or heartbeats.
Initial reference is the first teleop Home command/start time. Clear on lifecycle
restart. Disabled/default configuration keeps legacy fixed-step behavior.
Existing source authority, sequence, hard position limits and freshness barriers
remain. Reject timestamp rollback. Fault diagnostics include side/delta/budget/dt.

This intentionally allows up to 0.20 rad across 50 ms only in the dedicated
simulation profile (4 rad/s). It is not an unrestricted bypass. A larger jump,
compressed-time catch-up burst or an old delayed proposal is not legitimate
merely because packets were dropped. Full PICO acceptance still requires a live
retest after restarting the old fault-latched process.

- [x] Add tests using the real coordinator proposal ingestion/command boundary:
  dropped intermediate samples, undelivered samples, repeated heartbeats,
  startup, excessive jump, capped long delay, rollback, lifecycle, legacy mode.
- [x] Observe .045 rad / .015 s scenario fail before implementation.
- [x] Implement per-side adopted-proposal time anchor and opt-in config window;
  preserve legacy clipping and validate configured window <= proposal timeout.
- [x] Run coordinator/profile/protocol regressions and isolated direct sim smoke;
  document evidence. Do not change original v131 numerical integration.

## Verification and scope

- Before: /tmp/v131-timing-red.log confirms the legitimate multi-cycle command
  faults. Timestamp rollback/config validation regressions also fail before fix.
- Added separate red/green test for a stationary solver-failure hold: keep the
  last model-reference timestamp when holding an unchanged final command.
- 14 targeted timing tests include 120 complete coordinator ticks, dropping two
  out of three proposals, and verify exact direct command adoption. Malformed
  timing/over-speed/stale inputs still fault; legacy guard unchanged.
- /tmp/v131-timing-regression.log: 139 focused tests passed, no skips.
- First direct isolated smoke: /tmp/pico-sim-smoke-ctw1bc65/result.json, passed.
- Final direct isolated smoke after failure-hold correction:
  /tmp/pico-sim-smoke-2d7xmnns/result.json, passed with both overlays and no hands.
- No C++/IK/model/scheduler changes or rebuild in this fix; only coordinator,
  its dedicated config, tests and docs. Existing active processes were not
  restarted or sent control messages. No commits or subagents.
- Live fault was observed in run 03a38838-054a-49b6-ade7-39e51a63191a. No trace
  of that exact fault-triggering proposal was available, so timestamp continuity
  is the reproduced integration defect, not a claim to have proven the precise
  scheduling history of that live failure. Retest real PICO after session restart;
  new step_rejection diagnostics expose evidence if another path trips safety.
