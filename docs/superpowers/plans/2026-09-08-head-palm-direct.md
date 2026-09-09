# Independent head_palm_direct mapping

User approved a new option while retaining relative_home and head_direct.
Input remains PICO2 head-relative Wrist pose, not skeleton Palm extraction.
Sequential implementation, no agents, commits, hardware or user-session restart.

Design: subclass HeadDirectMapper with independently configured base geometry.
Postmultiply TCP rotation by local Rz(correction_deg); translate target by
R_base_head * [0,0,head_height_offset_m]. Local Z correction preserves the
existing 36.5 mm control-center offset (on local Z) and does not orbit the target.
User-confirmed corrections: left +90 degrees, right -90 degrees about local Z.
Height offset remains -0.10 m, a provisional simulation setting, NOT measured
calibration. Physical alignment and the actual height error require validation.
Parameters document their axes/units; old configurations are not reused by reference
or mutated. No IK, raw observation, hand retarget or hold-policy changes.

- [x] Write failing tests for factory/config selection, local rotation composition,
  height on virtual-head Z (not arm Base Z), invalid finite values, old config
  isolation, source status, and holding/recovery with original fixed mapping.
- [x] Register HeadPalmDirectMapper; add separate head_palm_direct_mapper_config
  with all fixed geometry explicitly copied, plus corrections and height.
- [x] Wire startup --arm-pose-mapper head_palm_direct and strict config validation;
  retain simulation/PICO restrictions and old defaults.
- [x] Run focused regression and isolated native v131 smoke with tracking loss.
- [x] Document command, assumptions and adjustment fields; self-review and record
  results. No claim of physical calibration correctness from synthetic tests.

Verification: initial tests rejected missing factory/backend selector before code.
Final focused suite: 133 tests, 132 passed, 1 skipped (`/tmp/head-palm-regression.log`).
New suite has six tests; existing head_direct, relative_home, hold and source tests
remain green. Local-Z rotation is tested with nontrivial input rotations, and
the 36.5 mm local-Z control point is unchanged by orientation correction.
Full-process v131 simulation with new mapper and unilateral/bilateral loss/recovery:
`/tmp/pico-sim-smoke-ngadniq_/result.json`, passed.
Shell syntax and git diff whitespace checks passed. No native IK edits or rebuild,
no commits or subagents. Physical correction signs/axis and height remain provisional.
