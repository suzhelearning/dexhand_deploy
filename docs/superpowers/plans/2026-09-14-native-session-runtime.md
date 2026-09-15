# Native session state machine and scheduler implementation plan

**Goal:** move session decisions and fixed-rate ownership into C++, without changing the two verified live routes.

**Architecture:** a transport-independent C++ session machine consumes validated health facts and explicit operator events. A C++ thread owns that machine, absolute deadlines, bounded operator events and result snapshots. The first integration is offline: no Python callback on each tick and no robot command publisher. Live ingress, IK/command ownership and recordings must subsequently be connected before this can replace the existing runtime.

**Tech stack:** C++17, `std::thread`/`condition_variable`, standalone C++ tests and differential Python harness using the current coordinator as reference.

## Constraints

- Work sequentially in the current branch; preserve all existing modifications, no subagents, commit or push.
- No devices, live sessions, network listeners or robot output during this work.
- Default live state machine and scheduler remain unchanged until integrated equivalence checks pass.
- Health facts are outputs of validated ingress, not public motion authorizations. A boolean-facts test driver is not a production input protocol.
- Explicit start only, fault cannot be cleared by rearm, exact Home and acknowledged reset required for rearm, fresh input required afterward.
- Interrupted shutdown is not successful Home completion; bounded-queue failure must be visible and fail closed.

## Updated scope: retain hardware drivers, migrate application hot paths

The user explicitly retains ALL existing hardware drivers, SDKs and device connection interfaces,
not only the real-arm driver. Do not rewrite/reconfigure the PICO or Manus SDK, arm/hand drivers,
USB access, device bindings or head-mounted APK as a consequence of this migration.
The C++ boundary starts at their existing output contract; no new permission for physical motion.

Remaining implementation must deliver connected application paths, not only standalone native headers:

- [ ] **Input parsing:** consume the unchanged driver outputs in C++; preserve PICO2 and VR/Manus
  packet layouts, timestamps, identities, source generations, skeleton point order and rejection gates.
  Compare accepted/rejected frames against the Python reference, including reconnects and truncation.
  - [x] Optional `--manus-parser-backend cpp` connects the post-rawviz parser and callback assembler
    to the existing receiver. Preserve semantic point selection, float32/Y reflection, per-side invalidation,
    duplicate rejection and right-first output. Keep original SDK/receiver lifecycle and default Python parser;
    native resources are bounded and overflow fails visibly. Full native transport ownership remains pending.
- [ ] **Hand retarget/filtering:** port the pinned official Wuji2 algorithm and its filter state,
  per-side validity, gap reset, URDF clamp and joint-name permutation. Preserve the Manus callback order
  and PICO2 independent-side behavior. Compare both raw recorded inputs and resulting joint sequences;
  a C++ wrapper still calling the Python retarget worker is not completion of this task.
  - [x] Port the official 20-DOF LPFilter state/update/reset to an ABI-independent C++ library;
    connect an explicit optional backend in the existing worker for both PICO2 and Manus.
    Preserve float32 rounding and float64 promotion, compare the pinned worker outputs and record
    backend/library provenance. The optimizer, geometric preprocessing and complete hand ownership remain pending.
  - [x] Add optional native geometric preprocessing in the existing official worker: input reflection,
    SVD wrist frame, side basis, configured rotation and offsets. Compare nondegenerate reference frames
    and complete worker results, record provenance; explicitly reject non-unique degenerate frames.
    The full optimizer and application ownership remain pending; defaults unchanged.
  - [x] Add independently selectable native AdaptiveOptimizerAnalytical with pinned Pinocchio/NLopt:
    target construction, FK/Jacobians, analytical objective/gradient, SLSQP callbacks, float32 warm state.
    Keep the original Python optimizer untouched/default; compare both hands, optional penalties,
    reset/explicit initial states, complete workers, and saved Manus/PICO input samples.
    Python callback framing and full native application wiring remain pending.
- [ ] **Recording hot path:** connect raw ingress, full cycle/hand/audit records, shared timeline,
  explicit publication metadata and bounded native queues to the C++ disk writer. Compare the entire
  populated HDF5 schema, failure markers and drain/abort behavior; partial column parity is insufficient.
- [ ] **Live control wiring:** join the native ingress, retarget, state machine/scheduler, command
  ownership, IK, MuJoCo and output consumers behind explicit selectable application entry points.
  Keep Python outside the per-cycle path, preserve existing hardware-driver interfaces, and maintain
  old entry points for A/B regression. Native numeric helpers inside a Python loop alone do not satisfy this.
- [ ] **Acceptance:** run end-to-end offline equivalence, stop/Home/rearm/fault/gap tests and sustained
  load measurements before real-input MuJoCo acceptance. Real-device testing requires its own explicit
  device-operation request. Do not switch old defaults or claim speedup solely from offline unit tests.

Python/Shell may remain for configuration, process startup, dependency checks and offline analysis.
Work remains sequential, without subagents or commits/push unless separately requested.

## Implementation sequence

### 1. State-machine event contract

Files: `native/control/session_machine.hpp`, `tests/cpp/native_session_driver.cpp`, `tests/test_native_session_runtime.py`.

- [x] Write failing differential event tests for idle/start/repeated start/return/shutdown/fault and stale input; separately test rejected authority in C++.
- [x] Implement typed `SessionFacts`, `SessionState`, `SessionMachine::intent/tick/rearm` with nonnegative monotonic timestamps, reason, epoch and exact Home gates.
- [x] Compare transitions, acceptance and reasons to `ArmCommandCoordinator`; test native reset acknowledgment and fresh-input barrier separately as live-owner contracts. These tests do not compare command values or protocol envelopes.

### 2. Native scheduling ownership

Files: `native/control/session_runtime.hpp`, `tests/cpp/test_session_runtime.cpp`.

- [x] Test absolute deadline progression, explicit reanchor and integer overflow without wall-clock sleeps. Real-loop overrun stress remains pending.
- [x] Implement interruptible `std::thread` ownership, bounded event/reply queues, latest health snapshot and latched failure. No Python callback.
- [x] Test lifecycle/start twice/stop/wakeup, stale snapshot, shutdown awaiting Home and queue overflow; compile/run with ASan/UBSan.

### 3. Live replacement gate — not satisfied by an offline supervisor

- [x] Add `SessionCommands` under the native scheduler: paired typed proposal identity/run/epoch/tick/source-time checks, atomic numeric validation, final-command ownership, Home interpolation, fault hold and bounded command dispositions.
- [x] Differentially compare final joint values, transition reasons and disposition acceptance with the existing coordinator for direct/clipped tracking, Home and fault; test authority/queue/clock boundaries separately.
- [x] Add fixed-authority arms-only typed health/feedback validation under the C++ thread; disable external boolean facts in that mode, derive Home from canonical ordered finite feedback and preserve queue-admission freshness.
- [x] Compare 120 readiness/freshness/Home snapshots to the current coordinator; test replay, authority, observation-only source, joint order and queue-age boundaries in C++.
- [ ] Decode complete wire schemas, hand domains and remaining health/feedback fields; trusted `SessionFacts` is retained only in the earlier offline mode, and command disposition is not a complete public receipt.
  - [x] Add a receive-only native descriptor owner with local receive timestamps, bounded datagrams, truncation rejection and interruptible shutdown. Test via AF_UNIX socketpair only; binding a live UDP socket, component supervision and full recording ingress remain pending.
- [ ] Connect typed identity/epoch/freshness-validated ingress and the existing IK + numeric command kernel + MuJoCo under the native owner.
  - [x] Add opt-in C++ worker binary-step transport with typed complete result decoding, monotonic/consecutive request association, bounded cancellable I/O, and reset tick restart without clock rewind. Differentially test actual SPARK/mapped-palm workers against the Python client; no runtime scheduling or actuator wiring yet.
  - [x] Add independently owned C++ MuJoCo model/data, atomic joint batches and copied feedback/FK snapshots. Offline integration fixture connects worker, command coordinator, execution guard and simulator through Home shutdown; health/clock/operator input are fixtures, not production ingress or scheduled runtime.
  - [x] Connect owned simulation factory/apply/feedback/destruction to `SessionRuntime` thread; derive executor health and measured Home from this owner, reject external executor feedback/status, verify shutdown/interruption/fault cleanup on both models. IK production and its guard/reset lifecycle remain outside this scheduled mode.
  - [x] Remove temporary C++ vectors/full-state snapshot copying from paired arm apply/feedback via fixed-size arrays. Count operator-new calls, compare both model outputs, and preserve all-before-write validation and owner-thread rejection; this is not a whole-loop real-time qualification.
  - [x] Connect optional native IK factory, accepted latest raw sampling, internal proposals and execution guard under the scheduler. Verify both actual workers and late-result/stop/overflow/bad-result gates. The initial stage-14 scope permitted one start session; rearm/calibration are extended by the subsequent tasks below.
- [ ] Connect start/Home/rearm/calibration barriers with native worker reset acknowledgment and new input, rather than a synthetic acknowledgment.
  - [x] Add direct C++ process/IPC reset transport and verify actual SPARK and mapped-palm worker state acknowledgments, bounded failure handling and child cleanup.
  - [x] Connect actual reset endpoint through a cancellable C++ task; enforce pre/post-reset healthy exact Home and matching worker/session epochs, commit only after verified completion.
  - [x] Verify scheduler progress, stale/moved feedback, mismatched epoch, cancellation, actual SPARK/mapped-palm session commit and blocked start after reset/heartbeats.
  - [x] Connect optional raw TJVR acceptance through the imported decoder/stream gate and mapped-palm selector; require a post-commit fresh frame in the captured epoch/generation to release the barrier. Verify actual worker resets and explicit start, including duplicate frames, changed epochs and second rearm. This is local datagram submission, not a live UDP receiver or IK step.
  - [x] Extend owned IK mode with its borrowed native reset service. Keep the scheduler at measured exact Home while resetting asynchronously; commit coordinator epoch, execution guard and sampled-input barrier only after actual ACK and unchanged fresh source generation. Join reset before destroying IK on cancellation/fault. Test both actual workers through restart and controlled reset failures. Height calibration is handled separately in the next task.
  - [x] Connect opt-in mapped-palm height sampling, read-only model reference, reset+height ACK transaction and post-commit new-frame/start gate. Preserve old committed offsets on sampling failure; fail closed after IPC/configuration failure. Differentially test sampling/reference and actual worker persistence; test cancellation and ACK failure.
- [ ] Preserve authorized hand output, bounded recording/diagnostics and exclusive command ownership; compare complete commands, receipts and HDF5.
  - [x] Preserve generation-time epoch/tick/time/run/router in internal receipts; add output-side complete bilateral receipt encoding and compare all fields with the Python coordinator for normal/clipped/fault commands. Publisher/recorder lifecycle wiring remains pending.
  - [x] Add a bounded native FIFO consumer with explicit drain/finalize versus abort, visible overflow/write/finalize errors and concurrent cancellation. Combine receive thread, original TJVR gate, runtime and receipt consumer in an offline fixture; no publisher authority or session HDF5 completion is inferred from queue completion.
  - [x] Connect an independent local failure lane and a sole native reply/receipt pump. Output overflow/write errors fault runtime; receive failure can report through the same lane. Discard IK results arriving after failure and cancel pending reset without committing epoch. Separate drained output closure from Home-complete session finalization. Concrete HDF5/Zenoh sinks and full input audit remain pending.
  - [x] Add native HDF5 column encoding and a bounded/cancellable UNIX-stream client for the existing disk worker. Compare audit/raw TJVR schema 1.2 contents, types and attributes to the canonical writer; test numeric/vlen fields and transport faults. Schema initialization stays cold-path Python. This is not complete cycle snapshot wiring or permission to set complete before all producers drain.
  - [x] Add opt-in bounded completed-cycle capture: value-owned state/command/feedback, source progress, complete worker request/result wire and adoption flag. Connect its sole consumer to the output pump; fault on overflow and preserve Home's final cycle. Verify actual workers across rearm and late-result rejection. Interrupted/exceptional unfinished cycles are not manufactured as complete; final runtime state remains mandatory. Full HDF5 column assembly, raw all-frame audit and hand domains remain pending.
  - [x] Encode the existing arm command/state/session-event HDF5 columns from completed snapshots with explicit owner metadata. Compare all dataset values, dtypes and attributes against the canonical writer and validate with its reader. Preserve absent velocities and nullable sequence flags; do not invent publication metadata. Full native_cycle audit, raw all-frame ingestion, hand domains and actual metadata-owner wiring remain pending.
- [ ] Add opt-in live entry only after end-to-end offline equivalence and lifecycle checks; real-input MuJoCo acceptance remains separate.

The native supervisor now also owns optional command generation, **not a fully migrated live controller**. Record actual verification and leave the remaining live gates open until implemented.

### 4. Optional native Hand2 worker/scheduler sub-chain (implemented, live owner still Python)

This stage is deliberately narrower than the full live-control checklist above. It migrates the
post-driver Hand2 worker and fixed-rate scheduler into an opt-in C++ process while retaining the
existing PICO/Manus device drivers and the Python transport/publication boundary.

- [x] Share one C++ pipeline between the legacy native worker and scheduler: geometry, pinned
  `AdaptiveOptimizerAnalytical`, LPFilter, finite/URDF limits and canonical 20-joint permutation.
- [x] Add fixed-size little-endian `TJHK/TJHS/TJHI/TJHO/TJHX` wire frames with explicit sizes,
  finite-value checks, sequence/timestamp/generation validation and no ABI-dependent struct layout.
- [x] Give the C++ scheduler ownership of fixed-rate ticks, latest accepted sample, bounded output
  queue, session phase, freshness, generation/epoch gates, reset callbacks and latched failures.
  The C++ thread has no Python callback and no device/Zenoh/robot/MuJoCo dependency.
- [x] Add a cold-only manifest launcher. The pinned Python environment resolves existing Hand2 YAML,
  model frame order and parameters once; the child then `exec`s the C++ scheduler.
- [x] Add explicit `--hand-scheduler-backend cpp` selection for PICO2 and VR/Manus simulation,
  retain `python` as the default, reject combination with `--hand-worker-backend cpp`, and record
  binary/source provenance in HDF5 metadata.
- [x] Preserve PICO2 independent left/right schedulers and connection generation; preserve VR/Manus
  right-then-left 126-point callback order. Explicit rearm and return-to-idle advance native epoch
  exactly once and require the next fresh input.
- [x] Verify the actual native process for idle/teleop, bilateral output, stale fail-closed,
  generation/epoch reset, output association, queue/wire bounds and normal Python-route isolation.
- [x] Verify both PICO2 and VR/Manus producer boundaries: independent PICO2 side schedulers
  preserve canonical joint names and connection generation; bilateral VR/Manus input produces
  both canonical command sides. Expired teleop frames are rejected before the stateful native
  pipeline runs, so stale input cannot advance filter/optimizer state.
- [x] Make the native scheduler pipe lifecycle cancellable: ignore SIGPIPE, poll bounded stdin/
  stdout I/O, wake input parsing after writer failure, and verify native children are reaped.
- [x] Add an opt-in asynchronous adapter: input submission no longer waits for a native solve;
  a dedicated reader drains fixed-rate results, coalesces only at the Python boundary, polls while
  the source is temporarily quiet, and rejects results crossing a session phase. PICO2 still
  requires a matched bilateral sequence before publishing a pair.
- [ ] Move raw PICO/Manus packet parsing, Python `HandRetargetLoop`, hand command publication and
  hand MuJoCo application into the same native live owner. These remain the next integration gate;
  the current C++ scheduler must not be described as an end-to-end C++ session controller.
  - [x] Extend the owned simulation endpoint with opt-in canonical left/right 20-joint feedback
    and atomic mixed arm/hand updates. Missing hands hold, invalid batches do not partially write,
    standard model names take priority over legacy aliases, and only the owner thread accesses
    model/data. Fixed-size hot-path arrays avoid C++ operator-new allocation in the measured loop.
    This execution seam does not itself admit or authorize hand commands; admission is handled
    by the following stage, and publication/rendering/complete recording remain pending.
  - [x] Connect optional bilateral hand input receipts and native scheduler results to
    `SessionRuntime`: fixed source/producer/run binding, bounded input association, monotonic
    sequence/generation, phase/epoch fences, source-time freshness and finite joint limits.
    Derive readiness from processed input plus owned model feedback. Hold on fault, reset to
    configured zero only on explicit simulation return, and discard pre-start/pre-rearm results.
    Verify native scheduler-thread output through the runtime and both MuJoCo models using
    offline numerical fixtures. Public wire/driver-output ingestion and hand output consumers
    remain pending; this does not enable full hands in the existing native gateway CLI.
  - [x] Add opt-in `TJHR`/`TJHA` native hand reset request/completion frames and a
    single-owner C++ child IPC client. Commit the client epoch only after an exact
    request-associated acknowledgement from both completed solver resets; reject
    malformed/truncated replies and bound timeout/cancellation. Legacy `TJHS` clients
    receive no additional frames. Verify against the actual native Hand2 process.
  - [ ] Wire this client into the main session's coordinated arm/hand reset transaction
    and live ingress, then complete hand publication/viewer/HDF5 fan-out before
    removing the public arms-only restriction. The standalone client is not this wiring.
    - [x] Add optional `HandResetEndpoint` to `SessionRuntime`, requiring both actual
      arm and hand completed reset epochs before session commit. Keep exact-Home
      pre/postconditions, cancellation of both services and post-commit new-input gates.
      Test actual Hand2 plus both actual arm worker processes; inject one-sided failure,
      unavailable hand endpoint and cancellation while the scheduler continues ticking.
      This internal reset integration does not connect live hand sample/result transport.
    - [ ] Connect hand input/result ownership and phase synchronization to that same worker,
      then full output consumers and cold live configuration. No public hands-enabled
      native scheduler option until this chain and its lifecycle are validated.
      - [x] Connect a typed normalized bilateral `HandSampleEnvelope` to an optional
        `HandPipelineEndpoint` in `SessionRuntime`. A separate C++ IPC owner serializes
        samples, phases and resets on one actual Hand2 child, with bounded FIFO queues,
        no main-tick IPC wait and no externally injected result authority. Validate actual
        hand execution on both models, Home/rearm/new-input gates, invalid generation,
        nonfinite input, stalled I/O, overflow and cancellation. Arm ingress/reset are
        explicit numerical fixtures in this test, not actual-arm-IK equivalence.
      - [ ] Attach unchanged driver outputs and PICO2 independent-side ingestion, full hand
        publication/viewer/HDF5 consumers and cold CLI selection. Compare full recorded
        trajectories and timing before exposing a hands-enabled native live route.

The opt-in build/selection is:

```bash
pixi run build-native-hand-scheduler
# append to a hands-enabled PICO2/VR-Manus simulation command:
--hand-scheduler-backend cpp
```

No default route is switched and the original Python worker/optimizer remains unchanged.
