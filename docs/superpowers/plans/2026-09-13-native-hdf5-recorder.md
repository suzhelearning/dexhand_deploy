# Native HDF5 recorder implementation plan

> Execute sequentially with executing-plans; no subagents and no commits.

**Goal:** Sustain VR/Manus recording without changing teleoperation or session schema 1.2.

**Architecture:** Reuse Python schema creation and validated message-to-column mapping, then close that file handle before transferring exclusive ownership to a C++ process. The dispatcher accumulates bounded column blocks, sends length-delimited binary messages, and receives one acknowledgement per block. C++ caches datasets and appends hyperslabs. PICO2 retains ProcessSessionWriter.

**Reference:** davinci `src/applications/common/hdf5/hdf5_session_io.cpp` and `ros_hdf5_record/apps/ros_hdf5_record.cpp`. Its finalized whole-session write is a reference, not a streaming implementation to copy. No runtime paths into davinci.

## Constraints

- Existing dataset names, dtypes, metadata, source sequences, audit events and replay remain unchanged.
- Exclusive creation; complete=false until successful drain/close. Failure remains fail-closed.
- Bounded buffers and IPC sizes, timeout/owned-child cleanup; no silent drops or implicit backend fallback.
- C++ runtime only receives recording data; no device, Zenoh or control authority.
- Keep existing latest-frame Manus filtering, arm IK and PICO2 backend unchanged.

## Tasks

- [x] Add native writer equivalence tests using existing raw callback/audit and TJVR fixtures; demonstrate missing implementation failure.
- [x] Add `native/hdf5_recorder/main.cpp`, a bounded column append protocol with cached HDF5 objects, scalar string attribute updates, flush/close/abort and error replies.
- [x] Add `scripts/build_hdf5_recorder.sh` and pixi task. Build locally against system HDF5; output under ignored build directory.
- [x] Add `recording/native_session_h5.py`: schema descriptors, immutable column staging, timed binary IPC, native lifecycle. Keep message validation inherited from the canonical writer.
- [x] Select native backend only for vr_manus_sim in AsyncDualRecorder; retain acquisition queue limits and fail-closed behavior. Report queue high-water mark and processed items.
- [x] Run complete dataset/attribute comparisons, native process failures, no-overwrite, abort/drain, replay integration, and full regression.
- [x] Run a sustained representative mixed-stream benchmark and document measured throughput/limitations, build instructions and device acceptance still required.

## Results (2026-09-13)

- Full regression before the guard/ADB review tests: 984 tests, no failures, 55 skips. Native/SPARK integration run: 22 tests passed; subsequent native/async lifecycle tests: 16 passed. Post-review full regression: 990 tests, 935 executed passes, 55 skips, no failures.
- Two 120-second runs at 2x mixed-stream rate, one with active IK/accepted-hand audit payloads. Active run: 47,999 control snapshots, 57,599 Manus callbacks, 21,599 TJVR packets, 11,999 bilateral hand updates; generated and saved counts match.
- Active run accepted/processed 357,590 queue items, high-water 147/16,384, final depth zero. 2,384 native column blocks; maximum observed IPC acknowledgement latency 49.09 ms. File complete=true.
- Synthetic artifacts: `recordings/device_acceptance/native_hdf5_stress_20260913_a.h5` and `native_hdf5_stress_20260913_active.h5`. These are stress-test outputs, not hardware acceptance recordings or algorithm replay fixtures.
- No devices opened and no control authority during benchmarks. Real-device long-duration combined teleoperation still requires user testing. No commits or pushes.

## Post-review Ctrl-C correction

The user's next device session operated normally but ended with Ctrl-C, an aborted Python process, and complete=false. Group-wide TERM was independently reproduced killing the disk child before Python could drain. Both VR launch layers now notify their session owner first (40/50-second grace), then retain existing group cleanup as fallback. Repeated interrupt signals are ignored during cleanup. The actual launcher/native-recorder integration test reproduces BrokenPipeError against the original launcher and passes with all 200 queued audit rows and complete=true after the fix. This establishes the signal-ordering fix, not a diagnosis of every possible SDK core dump; device teardown must be retested.

## Verification commands

```bash
bash scripts/build_hdf5_recorder.sh
PYTHONPATH=src/tianji_teleop pixi run python -m unittest tests.test_native_session_h5 tests.test_async_dual_recording -v
WUJI_REFERENCE_TEST=1 SPARK_NATIVE_TEST=1 PYTHONPATH=src/tianji_teleop pixi run python -m unittest tests.test_spark_live_simulation -q
PYTHONPATH=src/tianji_teleop pixi run python -m unittest discover -s tests -q
git diff --check
```
