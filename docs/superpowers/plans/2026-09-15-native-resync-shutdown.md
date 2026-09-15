# Native resynchronization and shutdown repair

Approved scope: preserve drivers, Python and PICO2 defaults; no commits.

## Shutdown

- [x] Add a regression with a continuous SIGPIPE-sensitive writer whose native reader exits before owner cleanup (reproduced exit -13 before repair).
- [x] Keep a non-consuming parent read descriptor until the owner terminates and waits for rawviz. Only the native receiver consumes bytes. Do not suppress process failures.
- [x] Run native Manus ownership and live shutdown tests, including genuine early exit detection.

## SPARK recovery

- [x] Add opt-in same-epoch resynchronization recovery without repeated fixed-joint takeover; retain original default, initial takeover and real epoch-change takeover.
- [x] Test default and opt-in behavior on discontinuities and epoch changes; preserve freshness/jump checks and controller constraints.
- [x] Wire `--spark-resync-policy reference|resume` through session configuration and worker argv; document and compare recorded input offline.

## Verification

64 Python/native-worker/CLI/adapter tests passed; 9 SPARK CTest groups passed.
Opt-in native joint integration passed both SPARK and mapped-palm subcases (synthetic input, C++ Viewer/HDF5, Home/rearm/shutdown).
Run tests with `PYTHONPATH=.:src/tianji_teleop`; child fixtures import the repository's tests package.

Replayed native cycles (including cycles without new input) from
`recordings/device_acceptance/native_joint_latest_20260915_022641_437731863.h5`
through the same SPARK worker using `--deterministic-test` in both modes:
6,285 cycles each, 11 resynchronizations each; fixed joint takeover cycles
reference=1,987, resume=303. This is a behavior comparison, not a latency or
tracking-accuracy qualification. Real-device subjective improvement and
normal-exit acceptance remain unverified after this repair.

Use sequential test-first implementation (writing-plans/test-driven-development); no subagents or hardware-driver edits.
