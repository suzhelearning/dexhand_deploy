# C++ Manus filter continuity repair

## Approved scope

Repair Manus native joint scheduling only. Preserve PICO/arm IK, all drivers,
Python reference, PICO2 default behavior and raw recording identities. No commit.

## Diagnosis

The latest-waiting-input scheduler intentionally skips old frames. SidePipeline
interprets nonconsecutive solver sequence numbers as optimizer/filter resets.
The Python live Manus caller already separates callback identity from solver
continuity, using a 200 ms timestamp threshold.

Recording `native_joint_resume_20260915_024618_127708988.h5` contained 17,145
accepted teleop hand results. Among adjacent results, 3,441 sequence gaps all
had timestamp gaps below 200 ms. Median maximum joint step: 6.78 degrees at
sequence gaps, 0.83 degrees otherwise (not a static-pose noise measurement).

## Implementation and verification

- [x] Add scheduler regression: normal skipped frames, exact threshold, timeout,
  independent missing-hand history, unchanged output source sequence.
  Observed failing assertion before implementing continuity remapping.
- [x] Add optional `filter_continuity_ns` (default zero/legacy). Solver-only
  sequences advance by one normally, by two after a timeout, inducing the existing
  pipeline reset. Explicit epoch reset clears both side histories. Raw input and
  result identity, freshness checks, generation checks and recording remain unchanged.
- [x] Enable 200,000,000 ns only in native Manus joint worker launch arguments.
- [x] Build native hand scheduler and verify C++ scheduler assertions.
- [x] 28 existing scheduler/worker/launch tests passed with WUJI_REFERENCE_TEST=1.
- [x] Additional real optimizer test against Python with skipped frames and timeout
  passed within 2e-5 rad for each joint on both sides.
- [x] Native joint integration passed SPARK and mapped-palm synthetic input subcases,
  including Home/rearm/shutdown and complete recording.
- [ ] Real-input subjective jitter acceptance after restart.

Commands: `pixi run build-native-hand-scheduler`;
`PYTHONPATH=.:src/tianji_teleop WUJI_REFERENCE_TEST=1 pixi run python -m unittest tests.test_native_hand_scheduler tests.test_native_hand_worker tests.test_native_hand_launch -q`;
`PYTHONPATH=.:src/tianji_teleop NATIVE_JOINT_GATEWAY_TEST=1 pixi run python -m unittest tests.test_native_joint_gateway -q`.
