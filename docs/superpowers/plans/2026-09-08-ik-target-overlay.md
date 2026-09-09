# IK target pose overlay

Scope approved by user: show the actual desired IK TCP poses in the existing
MuJoCo scene, similar to TJ_arm_control's target_L/R mocap markers. Passive only;
no solver tuning, control messages, commits, subagents or live-session restart.

Implementation uses bounded user_scn geometry rather than adding interactive
mocap bodies. Subscribe to actual canonical arm targets; validate source/router,
Base_L/R frame, timestamp and sequence. Derive world-from-base from the viewer's
fixed URDF chain, including stand translation and mirrored base rotations. No
second tracker mapping or wrist/TCP compensation. Preserve last target in grey
with a stale label after 0.5s, independent of control/retarget state.

- [x] Add failing tests for CLI, opt-in subscriptions, passive rendering,
  full-pose transform, stale target preservation, authority and sequence checks.
- [x] Verify computed base transforms against actual MuJoCo forward kinematics.
- [x] Add ik_target_overlay.py and integrate alongside raw/frame0 overlays.
- [x] Forward --ik-target-overlay only for hand_tracking simulation profiles.
- [x] Add per-side diagnostics and document display semantics.
- [x] Run isolated synthetic session with URDF limits and both overlays; confirm
  unchanged QP commands. Check offscreen rendering and final regression suite.

Validation: /tmp/pico-sim-smoke-_e7opt8k/result.json passed, both desired target
streams live, 962 unchanged proposal/command pairs. EGL offscreen rendering of
the actual robot with 116 raw+target overlay geometries was visually inspected.
No live-device IK fix is claimed by this diagnostic-display change.
