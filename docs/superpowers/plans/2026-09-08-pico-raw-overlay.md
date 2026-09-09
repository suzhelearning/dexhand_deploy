# PICO raw MuJoCo overlay

Goal: opt-in passive rendering of the raw headset, wrists and all 26 hand
joints, including when hand retargeting is disabled. No control-path changes.

Design: subscribe to raw/pico_hand_tracking only with --pico-overlay. Preserve
tracking-space FLU orientation, metre scale and relative geometry; display with
a fixed translation [0, 1.2, 0.8] in the MuJoCo scene. This is not robot registration
or an IK target. Draw RGB XYZ pose axes, left cyan/right orange bones, and a
labelled reference origin. Latest-frame cache is locked, router checked, ordered
within receiver/generation, and expires after 0.5 seconds. Invalid joints never
produce bones. Frame0 and PICO append to a scene cleared once per viewer tick.

Execution: sequential in the current worktree, no commits, no subagents,
no interaction with the running session (user constraints).

- [x] Add tests in tests/test_pico_raw_overlay.py; run them and confirm missing
  overlay support fails. Cover opt-in, unchanged qpos, raw geometry, stale data,
  wrong router, reordered frames, invalid joints and bounded real MjvScene.
- [x] Implement executors/mujoco/pico_overlay.py: raw cache, diagnostics and
  append-only geometry renderer. Integrate subscriber and render loop in node.py.
- [x] Forward --pico-overlay through run_session.sh; restrict to PICO simulation.
  Document the command and display coordinate semantics in README.md.
- [x] Run focused unittest regression suite and bash syntax checks; inspect diff.
  Live headset/viewer validation remains distinct from headless scene tests.

Validation (2026-09-08): 50 focused tests plus 33 hand-tracking tests pass.
The isolated synthetic session in /tmp/pico-sim-smoke-9tdg8u0w/result.json
passes with pico_ee_dexhand_qp, both passthrough processors, clipping false and
raw overlay enabled: 980 unchanged proposal/command pairs; both arms move.
An EGL offscreen render of the real model and 108 overlay geometries was
visually checked; reference text is offset above the origin to avoid overlap
with the initial headset position. No live headset or physical robot was used.
