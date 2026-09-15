# Unified live simulation Home

User-approved Home source: `config/robot/arm.yaml`, radians stored in YAML;
left [55,-65,-70,-60,60,0,0] degrees, right [-55,-65,70,-60,-60,0,0] degrees.

Python live VR+Manus and native gateway resolve the same arm Home while
retaining backend URDF bounds. Both SPARK and mapped-palm workers receive
`--home-config PATH` before constructing their initial controller state.
Existing reset/Home transactions use these same values; controller configuration
retains them across reset. No hardware drivers or target mappings are changed.

Offline reference harness and reference YAML retain their previous default
postures. This intentionally changes live startup from the original reference
baseline. Python/C++ refers to the live session scheduler, not every offline tool.

Tests cover arm.yaml resolution, native manifest forwarding, first worker cycle,
Python live Home and existing return/rearm paths. Native joint integration passed
both backends with complete recording. First-cycle comparisons allow 1e-5 rad:
the returned state is post-solver, and mapped-palm advances approximately 3e-6 rad
without a new target. Real-device revalidation after this change remains pending.

No commits and no live session started for this repair.

Final verification: Home/backend/live-core suite 20 tests (17 passed, 3 skipped);
worker-client/gateway adapter suite 16 tests (12 passed, 4 skipped); native joint
integration passed both backends. Expanded 54-test run hit one audit_clock_order
failure in test_live_headless_without_input_never_authorizes; isolated rerun
passed (19.762 s). This intermittent audit ordering issue is not claimed fixed.
