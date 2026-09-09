# PICO arm tracking loss hold, original mapping recovery

User chose recovery toward the original mapped wrist target, without reanchoring.
Sequential TDD, no agents/commits/hardware or restart of the active user session.

Scope: current PICO simulation with --disable-hands. Manus and hand-retarget
enabled profiles retain fail-closed observation behavior. While fresh authorized
PICO packets explicitly report invalid wrist tracking, keep teleop and freeze
only that arm's final joint command. Silence/stale packets, malformed identity,
frame changes and solver/limit failures are not converted to tracking loss.

Design: bridge emits optional tracking_valid=false on the arm target using a
cached pose only as wire placeholder; native producer MUST NOT solve that pose.
Producer resets IK/trajectory state and publishes tracking_hold proposals. The
coordinator freezes its own last final command (not delayed producer feedback).
Recovery retains the mapper reference and restarts IK from held joints. Optional
field defaults true and is omitted for normal targets, preserving old wire shape.

- [x] Tests: fresh loss keeps bridge/node teleop, original mapping recovery,
  stale loss still rejected, foreign/rollback invalid frames rejected, initial
  invalid cannot start; coordinator exact final-command hold and other-side motion.
- [x] Protocol/publisher optional strict boolean tracking_valid and native parser.
- [x] Bridge opt-in from PICO + no active hand sides; cached placeholder poses;
  source status reports tracking_hold_sides without unhealthy for fresh loss.
- [x] Native tracking hold branch bypasses solve, resets history, publishes hold;
  coordinator exact hold only in simulation, normal safeguards remain.
- [x] Extend isolated smoke with unilateral/bilateral loss and recovery assertions,
  no Home or session exit, held commands exact; build and run regressions.

## Verification / limits

- RED before native implementation: `/tmp/pico-sim-smoke-iu_1sfar`,
  timeout waiting for native tracking hold. Bridge/coordinator tests also failed
  before the corresponding implementation changes.
- Native build and installed probes passed: `/tmp/pico-hold-build.log`.
- Full-process tracking loss smoke passed: `/tmp/pico-sim-smoke-zq06knhi/result.json`.
  Tests unilateral hold with opposite-side motion, bilateral exact joint-command
  hold, automatic recovery without Home, and subsequent disconnect protection.
- Normal hands-enabled v131 direct-output smoke passed:
  `/tmp/pico-sim-smoke-y6ff6pl_/result.json`.
- Focused Python regression results: `/tmp/pico-hold-regression.log`;
  added tests also cover startup invalid, frame/identity/sequence rejection,
  source lifecycle, simulation-only hold, hard bounds and timestamp safeguards.
- Broader launcher suite is NOT green: `/tmp/pico-hold-launcher-tests.log`
  (27 tests; 9 failure reports, 2 errors), including unavailable
  `tianji_world_output`, missing `home.sh`, missing mock launch capture, and a
  script-text assertion forbidding the word `legacy` in the deployment comment.
  Those paths were not changed for this tracking-loss fix.
- Sequential self-review, no subagents or commits; no restart of user's session.
  Synthetic MuJoCo verification is not physical PICO dropout/latency acceptance.
  User must restart the old session to load the new source/native code.
