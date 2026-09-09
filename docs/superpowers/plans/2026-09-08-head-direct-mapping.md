# Head Direct Mapping Implementation Plan

Approved design: optional head_direct mapping, no start-pose subtraction;
relative_home remains default. Execute inline, sequentially, no commits or agents.

**Goal:** Select fixed head-to-base wrist mapping at session startup.
**Architecture:** HeadDirectMapper reuses DirectPoseMapper composition with strict
PICO head/wrist frame requirements. The target YAML holds separate fixed geometry;
launcher selects it through TIANJI_ARM_POSE_MAPPER. No IK/trajectory changes.
**Tech Stack:** Python/numpy/scipy, YAML, bash, unittest, isolated MuJoCo smoke.

## Contract

`T_base_tcp = T_base_head * T_head_wrist * T_wrist_tcp`.
initialize/reset never captures a wrist reference. s only authorizes teleop.
Tracking loss uses existing arm-only hold policy, recovery uses unchanged geometry.
Only PICO simulation supports this option; reject Manus and non-simulation profiles.
Fixed geometry is configurable, not a claimed reproduction of reference calibration.
Default virtual head is 0.30 m above the midpoint of the two base origins;
side rotations follow existing FLU-to-base conventions. Neutral wrist orientation
maps to configured robot Home orientation; TCP offset retains the 36.5 mm control
frame convention. These are nominal simulation settings, not measured human data.

## Tasks

- [x] Add tests in tests/test_head_direct_mapping.py: factory, exact composition,
  independence of initialization, frame rejection, config override/default,
  incompatible profile rejection and launcher help/invalid arguments.
  Run `PYTHONPATH=src/tianji_teleop:tests pixi run python -m unittest test_head_direct_mapping`
  and observe missing backend/selector failures before implementation.
- [x] Implement HeadDirectMapper in sources/common/pose_mapping.py, registered
  as head_direct; use DirectPoseMapper.map and replace backend/version labels.
  Reject frames other than pico_head_current/wrist, preserve full SE(3) composition.
- [x] Add head_direct_mapper_config to config/sources/hand_tracking_target.yaml.
  Add select_arm_pose_mapper(config, backend) to hand_tracking/target_node.py;
  validate separate config, select before constructing bridge, never alter the
  relative_home configuration or default selection.
- [x] Add --arm-pose-mapper {relative_home|head_direct} to scripts/run_session.sh
  (PICO simulation only), export selected value to source process.
  Pass the flag through scripts/pico_sim_smoke.py and check emitted status backend.
- [x] Run new tests and mapping/bridge/node/hold regression; isolated v131 smoke
  with head_direct plus tracking loss, preserving user live session.
- [x] Document full launch command, geometry calibration, startup motion and
  head-relative invariance versus independent head turns. Record verification.

## Verification

- Initial failing tests: `/tmp/head-direct-red.log` (backend, selector and CLI absent).
- Status integration RED: `/tmp/pico-sim-smoke-rf_yjxtk/result.json`
  (missing arm_pose_mapper diagnostic); fixed and verified in the subsequent run.
- Focused regression: `/tmp/head-direct-regression.log`, 127 tests,
  126 passed and 1 skipped. New mapper suite contains 9 tests, including common
  world-motion invariance, neutral TCP convention, startup independence and loss recovery.
- Head-direct full-process v131 smoke with tracking loss passed:
  `/tmp/pico-sim-smoke-3nkrydyu/result.json`.
- Default relative-home full-process v131 smoke with tracking loss passed:
  `/tmp/pico-sim-smoke-bv1um7uj/result.json`.
- `bash -n scripts/run_session.sh` and `git diff --check` passed.
- Sequential self-review completed; no native solver change/build needed,
  no user-session restart, hardware control, subagents or commits.
- Synthetic simulation only: real PICO wearing calibration and subjective tracking
  quality remain to be checked. Do not interpret nominal geometry as reference-project
  calibration parity or the regression set as all-project test coverage.
