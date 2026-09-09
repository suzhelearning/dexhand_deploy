# DexhandVelocityQpIk7 port

Historical archive: as of 2026-09-08, the public `pico_ee_dexhand_qp` backend
uses the v131 implementation documented in `../v131_qp/PORTING.md`.
The original core files and their probe remain for provenance only; they are
not selected by that factory name anymore.

Source: `/home/zj/current_robotics/TJ_arm_control_pico_ee_ik`, clean tree at
`3cfa5108b12d21232ce13a1f0d84831ad525d294`.

Original SHA-256 (source .cpp files are unchanged except for a provenance comment):

- `src/dexhand_velocity_qp_ik.cpp`: `6b08251331dab1608bd43e38dd960552e18f89a6afdb176f936724a69d67a81b`
- `src/dexhand_velocity_ruckig.cpp`: `23d33e6724462f6eb15f771194ec5d179dd65a175d321396b00f757c479b1e32`
- `src/so3.cpp`: `cfc7d4779bf5b1ef6b3741704897fe43d9447d40588495bd6c878b042e001285`

Minimal `tianji_qp_ik` headers retain only the types used by these components.
`KinematicsEvaluator` no longer depends on the source MuJoCo controller.
The pre-IK conditioning configuration member is excluded: this project owns
that independent stage in `ArmTargetProcessor`. The controller/SDK/application
from the source project are not copied.

`arm_ik.cpp` supplies current-project Base_L/R → TCP_Link_L/R poses and
base-aligned TCP Jacobians. The two arms have separate Dexhand solver state.
QP parameter names map to this project's `qp_*` settings; the new canonical
producer profile preserves source configured nominal joint poses.

The wrapped solver always uses its direct QP output. Optional Ruckig is owned
by the host producer, so the source internal limiter is never stacked with it.
The source limiter implementation is retained for provenance and future
explicit selection; current `--joint-trajectory ruckig` selects the existing
project `JointTrajectoryLimiter7`, not the source's hold-recovery variant.

Direct mode disables post-QP smoothing and coordinator clipping. It does not
disable QP velocity/position constraints, finite-value/hard-limit checks,
abnormal command-step rejection, source freshness, or bounded Home return.
No physical-device validation is implied by the simulation probes.
