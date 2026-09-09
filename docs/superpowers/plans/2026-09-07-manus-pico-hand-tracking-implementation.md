# Manus / PICO Hand Tracking Implementation Plan

> **Implementation status (2026-09-07):** Phase A and the simulation-only Phase B/C bridge are implemented in the working tree and verified with focused pure-Python, integration, configuration, recorder, and HDF5 tests. Changes remain uncommitted by request. The receive-only profile remains unchanged and real robot admission is still out of scope.

> **For agentic workers:** This plan was executed inline in the current workspace. Steps use checkbox syntax for tracking. The user requested sequential execution without subagents and without committing code.

**Goal:** Add a receive-only Manus/PICO observation pipeline that publishes a common 21-point hand skeleton, PICO current-head-relative wrist poses, legacy Manus-profile palm input, and extended session HDF5 records while preserving existing control and Regrind contracts. Then connect the canonical observations to the existing Wuji Hand 2 retarget and arm IK interfaces for an explicit simulation profile. The entrypoints are `pico_capture`, `manus_capture`, `hand_tracking_observation`, and the simulation-only hand-tracking session profiles.

**Architecture:** Pure protocol adapters decode native Manus JSON, PICO TCP, and legacy palm frames into typed observations. A runtime selects exactly one `input_profile` (`pico` or `manus`) and publishes observation/raw topics. A separate target bridge subscribes to those observations, applies the configured hand-frame adapter, pose-mapping factory, and target-processing factory, and publishes only the existing typed arm/hand target topics when the coordinator has authorized simulation teleop. Session HDF5 schema 1.1 is opt-in for the observation writer and passive simulation recorder, and remains backward-readable alongside schema 1.0. IK, coordinator, MuJoCo, and Wuji Hand 2 executors remain existing downstream components; no real admission is added.

**Tech Stack:** Python 3.10, NumPy, SciPy, h5py, PyYAML, eclipse-zenoh, existing unittest suite, TCP sockets, optional ADB executable.

## Global Constraints

- Do not run `git commit`, amend, reset, clean, checkout, or discard user changes.
- Do not start a robot executor, coordinator, IK producer, robot SDK, or control session from the new observation entrypoints.
- Do not use subagents; execute tasks sequentially in this workspace.
- `input_profile` selects exactly one complete profile; no automatic fallback or runtime hot switching.
- PICO hand protocol version 1 and joint count 26 are decoded only after strict header and payload validation.
- PICO mechanical-arm observation uses `pico_head_current`: `inverse(T_tracking_head) * T_tracking_wrist` from the same source frame.
- Manus 25→21 conversion uses semantic metadata; it must not guess fixed source indices when metadata is absent or ambiguous.
- The common hand skeleton order is MediaPipe 21 points, meters, wrist index 0, with per-joint validity.
- PICO raw records preserve head pose, flags, wrists, 26 joint poses, quaternions, radii, validity, and packet bytes.
- New HDF5 schema 1.1 must remain readable by the new reader without changing legacy schema 1.0 files.
- Existing `target/*`, `command/*`, and Regrind 123-observation/26-action contracts remain unchanged.

---

### Task 1: Freeze the pure common models and PICO transforms

**Files:**

- Create: `src/tianji_teleop/tianji_teleop/hand_tracking/__init__.py`
- Create: `src/tianji_teleop/tianji_teleop/hand_tracking/models.py`
- Create: `src/tianji_teleop/tianji_teleop/hand_tracking/pico.py`
- Create: `tests/test_hand_tracking_models.py`
- Create: `tests/test_pico_hand_tracking.py`

**Interfaces:**

- `HandObservation`, `ArmInputObservation`, and `PicoRawFrame` are immutable validated dataclasses with JSON-compatible `to_dict()` methods.
- `parse_pico_packet(packet: bytes, *, receiver_instance_id: str, connection_generation: int, receiver_frame_sequence: int, received_timestamp_ns: int) -> PicoRawFrame` decodes the known PICO_2 wire packet.
- `pico_to_mediapipe(joints, joint_valid) -> tuple[np.ndarray, np.ndarray]` returns `(21,3)` points and `(21,)` validity using indices `[1,2,3,4,5,7,8,9,10,12,13,14,15,17,18,19,20,22,23,24,25]`.
- `tracking_pose_to_current_head(head_pose, wrist_pose) -> np.ndarray` returns a 7-vector `[x,y,z,qx,qy,qz,qw]` in `pico_head_current`.

- [x] **Step 1: Write failing model and transform tests.**

Use unique numbered joint positions, verify the exact 26→21 map, verify per-joint invalidity, verify rigid common-frame invariance, and verify malformed quaternions/shapes are rejected.

- [x] **Step 2: Run the focused tests and observe the expected missing-module failure.**

Run: `PYTHONPATH=src/tianji_teleop python -m unittest tests.test_hand_tracking_models tests.test_pico_hand_tracking -v`

- [x] **Step 3: Implement the validated dataclasses, strict PICO packet parser, mapping table, and SE(3) current-head transform.**

The parser must reject wrong magic/type/version/joint count/short payload and preserve the exact packet bytes. It must retain source timestamp milliseconds as a source-clock field and never subtract it from PC monotonic time.

- [x] **Step 4: Run the focused tests and then the existing protocol tests.**

Run: `PYTHONPATH=src/tianji_teleop python -m unittest tests.test_hand_tracking_models tests.test_pico_hand_tracking tests.test_protocol -v`

- [x] **Step 5: Review the diff without committing.**

Run: `git diff --check && git status --short`.

### Task 2: Add strict Manus semantic conversion

**Files:**

- Create: `src/tianji_teleop/tianji_teleop/hand_tracking/manus.py`
- Create: `tests/test_manus_hand_tracking.py`

**Interfaces:**

- `parse_manus_payload(payload: Mapping[str, Any], *, receiver_instance_id: str, receiver_frame_sequence: int, received_timestamp_ns: int) -> ManusRawFrame` accepts the existing JSON fields and validates node metadata.
- `manus_to_mediapipe(raw: ManusRawFrame) -> HandObservation` resolves required semantic keys and returns the common order with a versioned coordinate-frame label.
- Missing, duplicate, ambiguous, non-finite, or out-of-range semantics produce an invalid derived result with a diagnostic error; no fixed-index fallback is allowed. The raw frame remains recordable when semantic metadata is incomplete.

- [x] **Step 1: Write failing shuffled-node, missing-semantic, duplicate-semantic, and coordinate-sign tests.**

The fixture must assign every required semantic a distinct coordinate so a numeric array-order assumption cannot pass accidentally. Include both sides and confirm the Manus-specific Y conversion is applied once.

- [x] **Step 2: Run the focused test and verify it fails for the missing adapter.**

Run: `PYTHONPATH=src/tianji_teleop python -m unittest tests.test_manus_hand_tracking -v`

- [x] **Step 3: Implement strict semantic indexing and raw-frame validation.**

Use the upstream `node_semantics` (`array_index`, `node_id`, `parent_id`, chain and joint type) and reject duplicate required semantic keys. Preserve source sequence, source timestamps, node positions, node quaternions, and metadata in the raw model.

- [x] **Step 4: Run Manus tests and Task 1 tests.**

Run: `PYTHONPATH=src/tianji_teleop python -m unittest tests.test_manus_hand_tracking tests.test_hand_tracking_models tests.test_pico_hand_tracking -v`

### Task 3: Add common observation protocol messages and topics

**Files:**

- Modify: `src/tianji_teleop/tianji_teleop/protocol/topics.py`
- Modify: `src/tianji_teleop/tianji_teleop/protocol/messages.py`
- Modify: `src/tianji_teleop/tianji_teleop/protocol/__init__.py` only if exports require it
- Create: `tests/test_hand_tracking_protocol.py`

**Interfaces:**

- Add `HAND_OBSERVATION`, `ARM_INPUT_OBSERVATION`, `RAW_PICO_HAND_TRACKING`, `RAW_MANUS_HAND_TRACKING`, and `RAW_LEGACY_PICO_PALM` topic constants with side helpers.
- Add strict `HandSkeletonObservation` and `ArmInputObservation` wire classes. Observation classes use `coordinate_frame`/`reference_frame`, independent validity, source/receive timestamps, receiver identity, frame association, and mapping version.
- Observation messages must not be accepted by `HandTargetCommand` or `ArmTargetCommand` parsers and must not publish on target topics.

- [x] **Step 1: Write failing round-trip, unknown-field, invalid-shape, invalid-quaternion, and independent-validity tests.**

- [x] **Step 2: Run the protocol tests to confirm the new message types and topics are absent.**

- [x] **Step 3: Implement strict schema-1 message classes and topic helpers.**

Use `strict_loads` behavior and reject NaN/Infinity. Represent invalid pose/keypoint values as `None` on JSON observations; retain masks for fixed arrays. Keep source and receiver frame identifiers distinct.

- [x] **Step 4: Run new and existing protocol tests.**

Run: `PYTHONPATH=src/tianji_teleop python -m unittest tests.test_hand_tracking_protocol tests.test_protocol -v`

### Task 4: Add the legacy palm adapter and configurable pose/target factories

**Files:**

- Create: `src/tianji_teleop/tianji_teleop/hand_tracking/legacy_pico.py`
- Create: `src/tianji_teleop/tianji_teleop/sources/common/pose_mapping.py`
- Create: `src/tianji_teleop/tianji_teleop/sources/common/target_processing.py`
- Create: `tests/test_pose_mapping.py`
- Create: `tests/test_target_processing.py`

**Interfaces:**

- `parse_legacy_pico_packet(data: bytes, ...) -> LegacyPicoPalmFrame` validates the committed `TJ_arm_control` TJVR v1-v4 protocol boundary and retains raw bytes. If the exact binary contract cannot be validated from the sender, the adapter exposes a strict configuration error instead of accepting guessed bytes.
- `create_arm_pose_mapper(name, config) -> ArmPoseMapper` registers `direct_pose` and `relative_home`.
- `create_arm_target_processor(name, config) -> ArmTargetProcessor` registers `passthrough` and `conditioned`.
- `direct_pose` performs explicit rigid input-reference/tracked-frame/TCP composition; `relative_home` preserves current `EndEffectorTargetMapper` relative behavior. `passthrough` does finite pose validation without changing geometry.

- [x] **Step 1: Write failing factory and behavior tests.**

Test unknown names, direct mapping without an initialization reference, relative mapping requiring initialization, processor pass-through exact equality, state reset, and no double-processing of orientation/scale.

- [x] **Step 2: Run focused tests and observe missing symbols/factory failures.**

Run: `PYTHONPATH=src/tianji_teleop python -m unittest tests.test_pose_mapping tests.test_target_processing -v`

- [x] **Step 3: Implement minimal pure factories and adapters.**

Keep current `EndEffectorTargetMapper` behavior intact until an explicit control migration task; factory code must be usable by observation preview without creating `TargetPublisher` or `SessionClient`.

- [x] **Step 4: Run focused and existing target-mapper tests.**

Run: `PYTHONPATH=src/tianji_teleop python -m unittest tests.test_pose_mapping tests.test_target_processing tests.test_target_mapper tests.test_canonical_sources -v`

### Task 5: Extend session HDF5 with schema 1.1 observation/raw groups

**Files:**

- Modify: `src/tianji_teleop/tianji_teleop/recording/session_h5.py`
- Create: `tests/test_hand_tracking_session_h5.py`
- Modify: `tests/test_session_h5.py` only if compatibility assertions need explicit schema 1.0 behavior

**Interfaces:**

- Keep `SCHEMA_VERSION == "1.0"` as the legacy default and add `EXTENDED_SCHEMA_VERSION == "1.1"`.
- `SessionH5Writer(..., schema_version="1.1", source_type="hand_tracking_observation", ...)` creates `raw/pico_hand_tracking`, `raw/manus_hand_tracking`, `raw/legacy_pico_palm`, `observation/hand_tracking/{left,right}`, and `observation/arm_input/{left,right}` while retaining standard groups for reader compatibility.
- Reader dispatch accepts schema 1.0 and 1.1 with exact group validation. Existing 1.0 files remain unchanged.
- Add append/read methods for raw and observation dataclasses, preserving masks, nullability, source/receiver IDs, mapping/reference metadata, packet bytes, and frame association.

- [x] **Step 1: Write failing schema 1.1 layout and round-trip tests.**

Verify PICO all 26 joint fields, packet bytes, current-head wrists, independent hand/arm validity, Manus raw semantic metadata, invalid fixed-array masks, `complete` behavior, and schema 1.0 read compatibility.

- [x] **Step 2: Run the focused HDF5 test and observe unsupported-schema/layout failure.**

Run: `PYTHONPATH=src/tianji_teleop python -m unittest tests.test_hand_tracking_session_h5 -v`

- [x] **Step 3: Implement schema-version-specific layout, append, reader validation, and read methods.**

Use explicit masks and fill values for invalid fixed arrays. Do not put observations under `target` or `joint`. Keep complete=false on abort and reject unsafe links.

- [x] **Step 4: Run HDF5 tests and all existing session tests.**

Run: `PYTHONPATH=src/tianji_teleop python -m unittest tests.test_hand_tracking_session_h5 tests.test_session_h5 tests.test_session_recorder tests.test_h5_replay -v`

### Task 6: Add receive-only PICO runtime and standalone entrypoint

**Files:**

- Modify: `src/tianji_teleop/tianji_teleop/hand_tracking/pico.py` for stream buffering if required
- Create: `src/tianji_teleop/tianji_teleop/hand_tracking/runtime.py`
- Create: `src/tianji_teleop/scripts/pico_capture`
- Modify: `tests/test_hand_tracking_runtime.py`
- Modify: `README.md` with the receive-only command and safety behavior

**Interfaces:**

- `PicoTcpReceiver` owns a socket, parses TCP framing, handles partial/coalesced packets, increments connection generation, and closes the socket to stop cleanly.
- `ObservationRuntime(...)` publishes raw and derived observations for the selected source callback and optionally appends schema 1.1 HDF5; profile selection is enforced by the observation CLI/config layer.
- `pico_capture --config PATH [--record PATH] [--no-adb-forward]` never opens a Zenoh target/command publisher and never imports coordinator/executor modules.
- ADB forwarding is injectable and optional; tests use a fake socket/clock and never invoke ADB.

- [x] **Step 1: Write failing stream parser/runtime tests.**

Cover partial TCP reads, two packets in one read, malformed packet recovery, disconnect generation reset, head-relative wrist output, and assertion that target/command topics are never declared.

- [x] **Step 2: Run focused tests and confirm the entrypoint/runtime is absent.**

Run: `PYTHONPATH=src/tianji_teleop python -m unittest tests.test_hand_tracking_runtime -v`

- [x] **Step 3: Implement receiver, runtime, optional ADB forwarding, status, and standalone script.**

The script may use Zenoh observation/raw topics and schema 1.1 recorder only. It must fail closed when `observation_only` is false and must not start a control session.

- [x] **Step 4: Run focused tests and shell syntax checks.**

Run: `PYTHONPATH=src/tianji_teleop python -m unittest tests.test_hand_tracking_runtime -v` and `bash -n src/tianji_teleop/scripts/pico_capture`.

### Task 7: Add integrated observation profile with Manus/PICO selection

**Files:**

- Modify: `src/tianji_teleop/tianji_teleop/hand_tracking/runtime.py`
- Create: `src/tianji_teleop/config/sources/hand_tracking_observation.yaml`
- Create: `src/tianji_teleop/config/sessions/hand_tracking_observation.yaml`
- Modify: `src/tianji_teleop/scripts/run_source.sh`
- Modify: `scripts/run_session.sh`
- Modify: `scripts/doctor.sh` only for new observation profile validation
- Create: `src/tianji_teleop/scripts/hand_tracking_observation`
- Modify: `tests/test_task8_config_launcher.py`

**Interfaces:**

- Integrated observation entrypoint accepts the source profile and uses the same `ObservationRuntime` as `pico_capture`.
- `input_profile: pico` activates PICO_2 hand/TCP input only. `input_profile: manus` activates Manus Zenoh JSON plus the legacy palm receiver configured by explicit endpoint/port.
- The launcher does not require `TIANJI_COORDINATOR_INSTANCE_ID`, does not launch coordinator/IK/executor, and does not create `target/*` or `command/*` publishers.
- `run_session.sh --profile hand_tracking_observation` executes only the observation process and optionally the HDF5 recorder; it rejects control-only flags and does not enter the existing control startup path.

- [x] **Step 1: Write failing launcher tests.**

Test profile validation, exactly-one input profile, missing Manus legacy endpoint, no coordinator requirement, no control process construction, and PICO/Manus mutual exclusion.

- [x] **Step 2: Run launcher tests and observe missing profile/entrypoint behavior.**

Run: `PYTHONPATH=src/tianji_teleop python -m unittest tests.test_task8_config_launcher.Task8ConfigTreeTest.test_hand_tracking_profiles_are_receive_only_and_select_one_input_profile -v`

- [x] **Step 3: Implement the observation config, source wrapper, and isolated launch branch.**

Keep the existing `mocap_live` control profile unchanged. `hand_tracking_observation` is the integrated receive-only entry; `pico_capture` remains the direct PICO entry.

- [x] **Step 4: Run launcher tests and shell syntax checks.**

Run: `PYTHONPATH=src/tianji_teleop python -m unittest tests.test_task8_config_launcher.Task8ConfigTreeTest.test_hand_tracking_profiles_are_receive_only_and_select_one_input_profile -v` and `bash -n scripts/run_session.sh scripts/run_source.sh src/tianji_teleop/scripts/hand_tracking_observation src/tianji_teleop/scripts/pico_capture`.

### Task 8: Add phase-A validation and finalize documentation

**Files:**

- Modify: `docs/manus_pico_hand_tracking_design.md`
- Modify: `README.md`
- Create: `tests/test_hand_tracking_integration.py`

**Interfaces:**

- Integration tests use fake PICO/Manus frames and assert raw/derived observation association, source selection, current-head invariance, and no control-topic publication.

- [x] **Step 1: Write failing integration tests for both profiles.**

- [x] **Step 2: Run them to establish missing integration behavior.**

- [x] **Step 3: Implement only documentation/test harness adjustments needed by the actual runtime.**

- [x] **Step 4: Run focused suite, then the full available Python suite.**

Run: `PYTHONPATH=src/tianji_teleop python -m unittest tests.test_hand_tracking_models tests.test_pico_hand_tracking tests.test_manus_hand_tracking tests.test_hand_tracking_protocol tests.test_pose_mapping tests.test_target_processing tests.test_hand_tracking_runtime tests.test_legacy_pico_palm tests.test_hand_tracking_integration -v`; run HDF5 tests with the PICO tracker environment because system Python lacks h5py.

If the repository's managed `scripts/test.sh` requires an unavailable external router/ACL, report that environmental limitation separately after running the focused suite; do not start a robot session.

---

## Phase B/C: Simulation target bridge and existing downstream wiring

The following tasks extend the approved Phase A plan. They deliberately stop at
simulation: the new bridge can publish typed targets only after the normal
coordinator session has authorized teleop, while the launcher rejects real
capability and never starts a real executor.

### Task 9: Add source-specific hand target adaptation

**Files:**

- Create: `src/tianji_teleop/tianji_teleop/hand_tracking/hand_target_adapter.py`
- Create: `tests/test_hand_target_adapter.py`
- Modify: `src/tianji_teleop/tianji_teleop/hand_tracking/__init__.py` only if exports require it

**Interfaces:**

- `create_hand_target_adapter(name, config)` returns a strict adapter for the
  canonical 21-point observation.
- The adapter validates source, coordinate-frame, side, router and freshness
  metadata, applies one configured proper rotation to the wrist-relative points,
  and returns the exact `wrist_relative_mediapipe` target geometry.
- Invalid observations never become targets. PICO does not inherit Manus's
  Y-axis correction; source-specific rotations remain explicit in config.

- [x] **Step 1: Write failing tests** for profile/frame allow-lists, one-time
  rotation, exact wrist origin, source identity/sequence rollback, stale and
  invalid observations.
- [x] **Step 2: Run the focused test and observe the missing adapter.**
- [x] **Step 3: Implement the pure adapter without Zenoh or session lifecycle.**
- [x] **Step 4: Run the focused hand adapter and existing hand protocol tests.**

### Task 10: Add observation-to-target bridge with session authority

**Files:**

- Create: `src/tianji_teleop/tianji_teleop/hand_tracking/target_bridge.py`
- Create: `tests/test_hand_tracking_target_bridge.py`
- Modify: `src/tianji_teleop/tianji_teleop/sources/common/pose_mapping.py` only for verified seam defects
- Modify: `src/tianji_teleop/tianji_teleop/sources/common/target_processing.py` only for verified seam defects

**Interfaces:**

- The bridge consumes typed `HandSkeletonObservation` and
  `ArmInputObservation` messages and uses injected publishers/callbacks, so it
  can be tested without a router, IK process, executor, or robot device.
- On an authorized teleop tick it publishes `HandTargetCommand` and
  `ArmTargetCommand`; arm targets pass through `create_arm_pose_mapper` then
  `create_arm_target_processor`, including `elbow_reference_direction`.
- `relative_home` initializes from the first valid arm observation per side;
  `direct_pose` does not invent a reference. Any source/side/frame/sequence or
  freshness failure causes a fail-closed return request rather than stale target
  replay.

- [x] **Step 1: Write failing pure bridge tests** for PICO relative-home,
  Manus direct-pose, passthrough processing, hand/arm association metadata,
  invalid/stale input, and no target before teleop authorization.
- [x] **Step 2: Run the focused test and observe the missing bridge.**
- [x] **Step 3: Implement the bridge with injected clock and publication sink.**
- [x] **Step 4: Run bridge, pose-mapping, target-processing, and protocol tests.**

### Task 11: Add an explicit simulation control entrypoint

**Files:**

- Create: `src/tianji_teleop/tianji_teleop/hand_tracking/target_node.py`
- Create: `src/tianji_teleop/scripts/hand_tracking_target`
- Create: `src/tianji_teleop/config/sources/hand_tracking_target.yaml`
- Modify: `src/tianji_teleop/scripts/run_source.sh`
- Create: `tests/test_hand_tracking_target_node.py`

**Interfaces:**

- `hand_tracking_target --config PATH` subscribes only to selected observation
  topics and publishes existing target topics only after coordinator-authorized
  teleop. It owns the normal `SessionClient` source lifecycle and publishes the
  source status required by the coordinator.
- It supports keyboard `s`/`q` lifecycle in an interactive session and an
  explicit diagnostic `--duration`; it never imports or starts IK/coordinator/
  executor code.
- The config selects exactly one `input_profile`, active sides, hand-frame
  adapter, arm pose mapper, target processor, and default elbow vectors.

- [x] **Step 1: Write failing node/config tests** for exact profile selection,
  observation subscriptions, target publication gating, session start/return,
  and absence of control-process construction.
- [x] **Step 2: Run the focused test and observe the missing node/config.**
- [x] **Step 3: Implement the target node and source wrapper.**
- [x] **Step 4: Run node tests and shell syntax checks.**

### Task 12: Add PICO/Manus simulation session profiles

**Files:**

- Create: `src/tianji_teleop/config/sessions/hand_tracking_sim.yaml`
- Create: `src/tianji_teleop/config/sessions/hand_tracking_sim_manus.yaml`
- Modify: `scripts/run_session.sh`
- Modify: `tests/test_task8_config_launcher.py`
- Modify: `README.md` and `docs/manus_pico_hand_tracking_design.md`

**Interfaces:**

- The simulation launcher starts, in order, coordinator, MuJoCo, Wuji Hand 2
  dry retarget executors, arm IK producer, target bridge, and the selected
  receive-only observation process with its source status suppressed. The
  target bridge is the sole coordinator source authority.
- `hand_tracking_sim` selects PICO by default and
  `hand_tracking_sim_manus` selects Manus plus the legacy palm stream. Both
  profiles use `required_capability: simulation`; `--confirm-real` and real
  executor configs are rejected.
- `hand_tracking_observation` remains receive-only and unchanged. No launcher
  test starts a router, MuJoCo, IK, or hand process.

- [x] **Step 1: Write failing launcher/config tests** for PICO and Manus
  profile selection, command ordering, single source authority, explicit
  simulation-only guard, and no real path.
- [x] **Step 2: Run the focused launcher tests and observe missing profiles.**
- [x] **Step 3: Implement the isolated launch branch.**
- [x] **Step 4: Run static launcher/config tests and shell syntax checks.**

### Task 13: Extend simulation-session recording and verification

**Files:**

- Modify: `src/tianji_teleop/tianji_teleop/recording/recorder.py`
- Modify: `src/tianji_teleop/tianji_teleop/recording/session_recorder.py`
- Create: `src/tianji_teleop/config/recording/session_hand_tracking.yaml`
- Modify: `scripts/run_session.sh`
- Create: `tests/test_hand_tracking_recorder.py`
- Modify: `tests/test_task8_config_launcher.py`
- Modify: `docs/manus_pico_hand_tracking_design.md`

**Interfaces:**

- A control simulation recording can retain schema 1.1 raw/observation streams
  together with typed arm/hand targets, while legacy schema 1.0 recorder
  behavior remains unchanged.
- Recorder selection is explicit; it never upgrades an existing file in place
  and never overwrites a path.
- Verification uses fake observation messages and typed target round-trips; no
  physical device or real robot is started.

- [x] **Step 1: Write failing schema 1.1 control-recording tests.**
- [x] **Step 2: Run them and verify the recorder currently rejects the profile.**
- [x] **Step 3: Implement explicit schema/profile dispatch.**
- [x] **Step 4: Run the full focused suite, HDF5 suite, and static checks.**

## Self-review checklist

- [x] Every plan task names exact files and a focused test command.
- [x] The plan separates device input, pose mapping, target processing, IK, and control launch authority.
- [x] PICO current-head semantics and raw packet retention are covered.
- [x] Manus raw semantic metadata and legacy palm input are covered.
- [x] Schema 1.0 compatibility and schema 1.1 strict layout are covered.
- [x] Receive-only entrypoints do not start a robot or publish target/command topics; simulation target publication is gated by the existing coordinator lifecycle.
- [x] No commit step is present because the user explicitly requested no commit.
