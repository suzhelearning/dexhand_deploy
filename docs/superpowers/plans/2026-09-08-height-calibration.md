# PICO horizontal arm height calibration

Approved: head_palm_direct only; c samples 2 seconds of stable, valid head-relative
Wrist positions, s starts teleop. No skeleton Palm input. No calibration in teleop,
returning or start_pending. Preserve other mappers and all IK/trajectory protections.
Sequential work, no commits, subagents, live-session restart or hardware commands.

Reference: URDF FK at q=0, explicitly horizontal lateral extension in this model;
verify shoulder/elbow/wrist world heights agree within 0.1 mm before accepting.
Use original IK TCP + local [0,0,0.0365] control point, transformed to Base.
Reference FK is read-only; robot never moves to the reference posture.
Projection uses world-up expressed in Base, checked against virtual-head up.

Calibrated control height = robot reference control height + input wrist z -
mean sampled wrist z. Recompute TCP from control center with its existing tool
offset; height calibration replaces the previous -0.10 m, never adds to it.
Forward and orientation unchanged. Per-side mean/offset applied atomically.
Optional c preserves old uncalibrated operation; calibration stays in memory for
this process including s return/restart, not persisted across process restarts.
Collection fails on invalid input, malformed input, freshness gap >0.25s,
fewer than 30 distinct frames/side, span <1.5s, or position range >0.06m.
Freshness uses received monotonic timestamps; repeated frames cannot calibrate.

- [x] RED tests: model reference, independent side means, control-height delta,
  unchanged XY/orientation, replaced old offset, collector gaps/motion/rollback,
  calibration/teleop gating and previous-result retention.
- [x] Implement isolated height_calibration.py sampler and URDF FK helper.
- [x] HeadPalmDirectMapper atomic height calibration API; accepted observation
  callback supplies samples directly, no bridge snapshot/accessor required.
- [x] Node c handling, collection callback/tick, status/log feedback, start gate.
- [x] Synthetic smoke c before s, validate source calibrated status, tracking loss.
- [x] Focused regression, docs and sequential self-review; no real PICO claims.

## Verification

Initial RED: missing module and c did not gate s. Additional RED tests exposed
a gap hidden by late tick and failed-first-calibration falling back silently.
Both were fixed: gap checked at receipt too; failed first attempt blocks s until
a successful retry. A failed retry with an existing successful result retains it.

Focused suite: `/tmp/height-calibration-regression.log`, 140 tests,
139 passed and 1 skipped. Six calibration tests cover reference rejection,
atomic geometry update, sample freshness/stability, lifecycle and failed retry.
Final full-process c/s plus unilateral/bilateral loss/recovery smoke passed:
`/tmp/pico-sim-smoke-651wicno/result.json`.
Earlier smoke also passed: `/tmp/pico-sim-smoke-_n1dr40i/result.json`.
`git diff --check` passed. No native IK modifications/build or real device actions.
Physical posture correctness and wearing alignment must be checked by the operator;
the sampler cannot prove horizontal/straight arms from wrist positions alone.

## Review fixes: failed-calibration recovery

Reproduced malformed arm input followed by a successful retry leaving the source
unhealthy. Added dedicated calibration input error state, separate from general
faults; only a successful calibration clears it. Unrelated errors are preserved
and prevent head_palm_direct start. Refresh source status before emitting start
intent. Existing mappers and all pose correction values are unchanged.

Shared `sources/common/control_geometry.py` now defines the original IK TCP to
control-center translation, consumed by both FK reference and calibrated mapping.
Numeric tests retain independent 36.5 mm expectations.

RED tests reproduced unhealthy retry and unrelated error overwrite. Final suite
`/tmp/height-retry-regression.log`: 142 tests, 141 passed, 1 skipped.
Full-process smoke now injects malformed observation ONLY on its isolated router,
checks failed/unhealthy then retried/calibrated/ready/healthy, starts with one s,
and exercises loss/hold/recovery and disconnect protection. Passed:
`/tmp/pico-sim-smoke-sxuce9wl/result.json`.
No user-session restart, native build, subagents, hardware actions or commits.
