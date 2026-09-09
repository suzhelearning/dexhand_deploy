# Joint limit source: implementation and validation

User-approved scope: add --joint-limit-source urdf to the existing Dexhand QP
simulation entrypoint. Sequential execution in this worktree; no commits or
subagents. Do not change the user's running session or drive physical hardware.

Design: do not remove downstream checks. Read the exact URDF passed to QP and
generate an exclusive per-session arm configuration snapshot. Preserve canonical
joint ordering and Home. Native producer/optional Ruckig, coordinator and MuJoCo
share TIANJI_ARM_CONFIG. Reject unsupported/asymmetric bounds and invalid Home.
Default/yaml mode retains previous config-source behavior; changing other solver
implementations and the independent return-completion bug are out of scope.

- [x] Tests first: URDF/YAML selection, missing/asymmetric joints, environment
  precedence, snapshot roundtrip, no overwrite, simulation-only launcher guard.
- [x] Add tianji_teleop/joint_limit_source.py for validated snapshot generation.
- [x] Honor TIANJI_ARM_CONFIG in ArmRobotConfig.load (explicit path wins).
- [x] Add run_session.sh option, validate profile/backend, pass shared snapshot.
- [x] Verify coordinator and MuJoCo use identical limits and preserve Home.
- [x] Document the complete command and remaining limitations in README.md.
- [x] Run isolated smoke with URDF option and verify actual native producer,
  coordinator and executor processes inherit the same snapshot, with QP direct
  commands unchanged. Run final regression tests and diff/shell checks.

Evidence: /tmp/pico-sim-smoke-_67qbssq/result.json reports passed=true,
970 unchanged command/proposal pairs and matching TIANJI_ARM_CONFIG in all
three actual process environments. Both arms moved. Native binary did not
require rebuilding: it already reads the passed config and its QP reads URDF.
