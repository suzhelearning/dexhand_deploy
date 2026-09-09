# v131 review fixes implementation plan

User approved the three review findings. Execute sequentially with executing-plans
and test-driven-development; no agents, commits, hardware or live-session changes.

Goal: preserve model-state IK fidelity while fixing optional processing and keeping
the legacy portable build independent of MuJoCo/qpOASES.

## Decisions

- Do not feed smoothed commands back into v131. Add a position-mode Ruckig entry
  point for independent model references; keep the existing velocity API unchanged
  for legacy solvers. Position targets use zero terminal velocity to avoid terminal
  velocity/limit conflicts; retarget each cycle, then catch up when the model stops.
- Preserve original failure classification: a solved-but-invalid QP becomes a
  solver failure after cold retry. Inject IHierarchicalQpSolver for deterministic
  fault tests. Distinguish normal acceptance from bounded fallback in the core;
  the host may deliver a bounded fallback with degraded status.
- Add TIANJI_ENABLE_V131, default OFF. The local simulation script explicitly
  enables it. The portable build explicitly disables it, preserving its existing
  ABI and packaging contract. A full portable build requesting v131 fails clearly
  until a portable runtime package exists; no silent absolute-path deployment.

## Tasks

- [x] Ruckig: extend joint_trajectory_limiter_probe with a fast moving position
  reference followed by a long hold; require final position and all derivative
  bounds. Add update_position(position), share validation/integration, wire only
  model_state_only to this API. Verify legacy probe behavior unchanged.
- [x] QP: inject a solver that first solves normally, then returns Solved with an
  invalid equality residual on both attempts. Require bounded fallback, nonzero
  integrated motion/history and normal recovery; also test cold-init failure.
  Update velocity_qp and adapter fallback semantics. Re-run original oracle traces.
- [x] Build: configure a separate simulation build with TIANJI_ENABLE_V131=OFF,
  require no MuJoCo/qpOASES DT_NEEDED and no v131 factory registration. First show
  the current unconditional dependency fails this check. Conditionalize sources,
  dependencies and probes; explicitly opt simulation in and portable build out.
  Guard deploy against staging v131 binaries. Run shell syntax and isolation tests.
- [x] Verify native probes, focused Python tests, original controller comparison,
  direct and smoothed isolated simulation smoke. Document exact scope and evidence.

Commands: pixi run -e ik-build bash scripts/build_ik_sim.sh;
pixi run -e ik-build check-ik-trajectory;
pixi run python scripts/compare_v131_reference.py --reference-root
/home/zj/current_robotics/TJ_arm_control_pico_ee_ik;
PYTHONPATH=src/tianji_teleop:tests pixi run python -m unittest test_v131_model
test_v131_profile test_arm_coordinator test_pico_arm_only test_joint_limit_source.

## Evidence

- Ruckig red: missing position API (/tmp/v131-review-ruckig-red.log); previous
  review's native velocity-only reproduction lost 1.2 rad of a 2 rad input.
  Position/velocity native probe now passes, including saturation catch-up,
  derivative limits and invalid-target state preservation.
- QP injection reproduced failure before the policy change ("solver validation/
  init failure did not preserve original bounded fallback"); v131_qp_probe now
  passes both solved-invalid and initialization failure, integrated-state recovery
  and all existing nominal/OTG/bound checks.
- Build/install plus four probes: /tmp/v131-review-build.log, passed.
- Disabled build: build/ik-legacy-review; /tmp/v131-review-legacy-build.log.
  Actual producer ELF has neither MuJoCo/qpOASES DT_NEEDED nor DexhandQpArmIk
  symbols; legacy PinocchioQpArmIk remains. Native limiter probe passed there too.
- Isolation tests: four failures observed before implementation, five tests now
  pass including actual deployment rejection in a disposable bundle before any
  cleanup/copy. Workspace deployment was not run.
- Focused Python suite: /tmp/v131-review-python.log, 125 passed, no skips with
  TIANJI_LEGACY_IK_BINARY=build/ik-legacy-review/arm_ik_producer.
- Original independent controller comparison:
  /tmp/v131-reference-compare-uo3t_kkk/result.json; 1200 bilateral frames, acceptance
  identical, maximum joint error 5.003653047452872e-11 rad. These finite traces do
  not constitute proof for arbitrary trajectories or live devices.
- Direct sim with both overlays: /tmp/pico-sim-smoke-9yu2ir41/result.json, passed.
- Conditioned + position Ruckig + clipping:
  /tmp/pico-sim-smoke-l2xhoki9/result.json, passed.
- Legacy pinocchio_qp session: /tmp/pico-sim-smoke-2wh0y3wt/result.json, passed.
- Shell syntax and git diff --check passed. No commits, agents or hardware runs.

Known pre-existing limitation rechecked: full joint_trajectory_limiter_probe with
URDF still fails its legacy Regrind moving target threshold (0.0144063 m vs 0.005 m),
as already recorded in README before this correction. Stationary error 0.000397055 m.
Evidence: /tmp/v131-review-legacy-dynamic.log. Do not claim all-product regression
qualification. Full portable build/deployment was not run: this workspace lacks
the original SDK/runtime bundle. Dependency isolation was verified using the
same native factory/producer in a separate v131-disabled simulation build.
