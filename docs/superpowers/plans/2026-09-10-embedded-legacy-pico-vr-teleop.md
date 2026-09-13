# Embedded Legacy PICO VR Teleop Implementation Plan

> For agentic workers: execute this plan inline and sequentially. Do not use subagents. Every task ends with a verification checkpoint.

**Goal:** Embed the original PICO driver, M0 corrected-skeleton pipeline, and TJVR bridge into the current repository and expose a managed pico_vr_manus_sim entry without changing the verified PICO2 route.

**Architecture:** Store the original pico_bridge source and its required launch/scripts under vendor/pico_tracker with a separate Python 3.11/ROS2 Pixi environment. A current-repository supervisor starts driver, raw readiness, M0, M0 readiness, and TJVR in that order, then delegates to the existing vr_manus_sim downstream. Existing pico2_hands_sim and manual vr_manus_sim remain separate.

**Tech Stack:** Bash process supervision, nested Pixi ROS2 Humble, original C++17 pico_bridge, current Zenoh/Spark/MuJoCo pipeline, unittest/pytest, ROS2 CTest.

## Global Constraints

- Preserve the original PICO semantics: ADB tcp:9999, M0 corrected topics, and TJVR default 127.0.0.1:15000.
- Do not change pico2_hands_sim, its PICO2 tcp:10002 input, official producer, hand conversion, or defaults.
- Keep ROS/Python 3.11 dependencies in the nested vendor/pico_tracker environment, not the root Pixi environment.
- Do not track APKs, private SDKs, personal calibration, recordings, .pixi, build, install, or log output.
- Do not put a development-machine absolute checkout path in tracked files.
- Preserve all pre-existing dirty worktree files; stage only files created or intentionally changed for this feature.
- Do not commit business-code changes during implementation.
- Write each new behavior test first, run it failing, then implement the minimum code.

---

### Task 1: Embed the original runtime source bundle

**Files:**
- Create: vendor/pico_tracker/.gitignore
- Create: vendor/pico_tracker/pixi.toml
- Create: vendor/pico_tracker/pixi.lock
- Create: vendor/pico_tracker/src/imu_ros2/
- Create: vendor/pico_tracker/src/pico_bridge/
- Create: vendor/pico_tracker/scripts/calibrate_pico_arm.sh
- Create: vendor/pico_tracker/scripts/calibrate_pico_palm_orientation.sh
- Create: vendor/pico_tracker/scripts/record_pico_tremor.sh
- Create: vendor/pico_tracker/scripts/start_pico_driver.sh
- Create: vendor/pico_tracker/scripts/start_pico_m0.sh
- Create: vendor/pico_tracker/scripts/start_tianji_pico_teleop.sh
- Create: vendor/pico_tracker/scripts/stop_tianji_pico_teleop.sh
- Create: vendor/pico_tracker/scripts/cleanup_tianji_pico_processes.py
- Create: vendor/pico_tracker/source_manifest.json
- Modify: .gitignore
- Test: tests/test_embedded_pico_bundle.py

**Interfaces:**
- The embedded source is a repository-relative path and contains the complete pico_bridge package required for driver, M0, and TJVR.
- The nested manifest has a locked build task named build-pico-bridge.
- source_manifest.json contains schema_version, source_commit, and sorted relative file hashes.

- [x] Step 1: Write the failing bundle test.

Create tests/test_embedded_pico_bundle.py with tests named test_complete_runtime_source_set_is_embedded, test_source_manifest_is_relative_and_hashed, test_nested_manifest_has_locked_build_task, and test_embedded_text_has_no_developer_checkout_path. The first test must require these paths: pixi.toml, pixi.lock, src/pico_bridge/package.xml, src/pico_bridge/CMakeLists.txt, src/pico_bridge/src/pico_bridge_node.cpp, src/pico_bridge/src/pico_smpl_ground_node.cpp, src/pico_bridge/src/tianji_mujoco_teleop_bridge_node.cpp, src/pico_bridge/launch/start_pico_bridge.launch.py, src/pico_bridge/launch/start_pico_palm_skeleton_filter.launch.py, src/pico_bridge/launch/start_tianji_mujoco_teleop.launch.py, scripts/start_pico_driver.sh, scripts/start_pico_m0.sh, and scripts/cleanup_tianji_pico_processes.py. The manifest test must reject absolute paths and hashes not matching ^[0-9a-f]{64}$.

- [x] Step 2: Run the test and verify the expected failure.

~~~bash
PYTHONPATH=src/tianji_teleop pixi run python -m pytest tests/test_embedded_pico_bundle.py -q
~~~

Expected result: failure caused by missing vendor/pico_tracker files, not a test collection error.

- [x] Step 3: Import the source without build artifacts.

Set PICO_REFERENCE_ROOT to the original PICO_tracker checkout. Copy the complete src/pico_bridge package, its declared imu_ros2 dependency, and the required root scripts into vendor/pico_tracker. Copy the original pixi.toml and pixi.lock. Add a build-pico-bridge task that runs colcon build with --base-paths src, --symlink-install, --packages-select imu_ros2 pico_bridge, and the original Python_ROOT_DIR/Python_FIND_VIRTUALENV CMake arguments. Do not copy .git, .pixi, build, install, log, recordings, APKs, or calibration directories.

Update .gitignore so vendor remains ignored except vendor/pico_tracker source files; explicitly ignore vendor/pico_tracker/.pixi, build, install, log, recordings, and generated runtime files. Generate the manifest with a script under scripts/generate_embedded_pico_manifest.py. It must hash sorted relative files, omit generated output and runtime directories, and use only the source Git commit value, never the source checkout path.

- [x] Step 4: Run the bundle test and diff check.

~~~bash
PYTHONPATH=src/tianji_teleop pixi run python -m pytest tests/test_embedded_pico_bundle.py -q
git diff --check -- .gitignore vendor/pico_tracker
~~~

Expected result: all bundle tests pass and only intended source files are present.

---

### Task 2: Add portable calibration paths and ROS readiness probes

**Files:**
- Create: scripts/generate_embedded_pico_manifest.py
- Create: scripts/embedded_pico_preflight.py
- Modify: vendor/pico_tracker/scripts/start_pico_driver.sh
- Modify: vendor/pico_tracker/scripts/start_pico_m0.sh
- Test: tests/test_embedded_pico_runtime.py

**Interfaces:**
- generate_embedded_pico_manifest.py --source-root PATH writes a deterministic source_manifest.json with relative paths.
- embedded_pico_preflight.py accepts --mode raw|m0|bridge, --timeout-s FLOAT, and --domain INT; it returns 0 only after all selected ROS topics receive a message and returns 2 after timeout.
- PICO_TRACKER_CONFIG_DIR overrides the default HOME/.config/pico_tracker for all runtime artifacts.

- [x] Step 1: Write failing tests.

Test that the copied driver and M0 scripts contain PICO_TRACKER_CONFIG_DIR, tracking_epoch_state_file, all left/right calibration filenames, and no development-machine checkout path. Test that invalid mode and domain return code 2 before rclpy is imported. Test the manifest generator rejects a path containing .. and produces identical JSON twice for the same source directory.

- [x] Step 2: Run the tests and verify failure.

~~~bash
PYTHONPATH=src/tianji_teleop pixi run python -m pytest tests/test_embedded_pico_runtime.py -q
~~~

Expected result: failure because the generator, preflight, and portable-script changes are absent.

- [x] Step 3: Implement the generator and preflight.

The generator must sort files, hash bytes using SHA-256, omit source_manifest.json and the runtime directories, and get source_commit through git -C when available or use embedded-source otherwise. The ROS preflight must use the exact topic sets below and the configured ROS_DOMAIN_ID. The driver owns only the raw SMPL stream; the palm TCP publishers start with M0, so palm topics must not be required before the M0 process exists:

~~~python
DRIVER_TOPICS = ("/pico/smpl_raw",)
RAW_TOPICS = DRIVER_TOPICS
M0_TOPICS = DRIVER_TOPICS + (
    "/pico/palm_left",
    "/pico/palm_right",
    "/pico/smpl_palm_corrected",
    "/pico/smpl_palm_corrected_ik",
    "/pico/smpl_palm_corrected/status",
    "/pico/tracking_epoch",
    "/pico/tracking_epoch/status",
)
BRIDGE_TOPICS = ("/pico/tianji_mujoco_teleop/status",)
~~~

Use rclpy subscriptions with the original message types and a monotonic timeout. Do not start a router, device, or robot from the preflight.

Patch the embedded driver and M0 wrappers to resolve config_dir from PICO_TRACKER_CONFIG_DIR or HOME/.config/pico_tracker. Preserve the original default. Pass the config_dir tracking epoch file to start_pico_bridge and pass left/right TCP, wrist, and geometry artifacts to the M0 launch exactly once.

- [x] Step 4: Run the focused tests.

~~~bash
PYTHONPATH=src/tianji_teleop pixi run python -m pytest tests/test_embedded_pico_runtime.py -q
~~~

Expected result: all tests pass.

---

### Task 3: Add the isolated pico_vr_manus_sim contract

**Files:**
- Create: src/tianji_teleop/config/sessions/pico_vr_manus_sim.yaml
- Modify: scripts/resolve_dual_session.py
- Test: tests/test_dual_session_config.py
- Test: tests/test_dual_session_launcher.py

**Interfaces:**
- resolve_dual_session.py --profile pico_vr_manus_sim --resolve-only returns a simulation contract with input_mode vr_manus, arm_input tjvr_corrected_palm, and receivers manus,tjvr.
- --disable-hands removes only the Manus receiver and sets active_hand_sides to an empty list.
- Existing pico2_hands_sim and vr_manus_sim resolution remains unchanged.

- [x] Step 1: Write failing tests.

Add test_embedded_pico_profile_resolves_without_processes. It must invoke the resolver, assert return code 0, assert config.input_mode is vr_manus, config.arm_input is tjvr_corrected_palm, config.receivers is ["tjvr"] with --disable-hands, config.active_hand_sides is [], and runtime_available is true. Add an assertion that the PICO2 profile still resolves to receiver pico2 and has no embedded profile marker.

- [x] Step 2: Run the tests and verify failure.

~~~bash
PYTHONPATH=src/tianji_teleop pixi run python -m pytest tests/test_dual_session_config.py tests/test_dual_session_launcher.py -q
~~~

Expected result: failure because the new YAML and resolver choice do not exist.

- [x] Step 3: Add the new YAML and resolver choice.

Use the existing reference-direct Spark contract: both active arms, both active hands, Spark backend, no pose mapper, official_wuji_hand2, passthrough target/trajectory, false command clipping, URDF limits, reference_execution_mode reference_direct, and rate_hz 200.0. Add the profile to resolver choices and report that embedded PICO runtime/device/calibration preflight is required. Do not add a new input mode or change PICO2 validation.

- [x] Step 4: Run the focused tests.

~~~bash
PYTHONPATH=src/tianji_teleop pixi run python -m pytest tests/test_dual_session_config.py tests/test_dual_session_launcher.py -q
~~~

Expected result: all focused profile tests pass.

---

### Task 4: Implement the embedded source supervisor

**Files:**
- Create: scripts/run_embedded_pico_vr_session.sh
- Modify: scripts/run_session.sh
- Test: tests/test_embedded_pico_launcher.py

**Interfaces:**
- run_session.sh --profile pico_vr_manus_sim delegates only this profile to the new supervisor.
- The supervisor accepts viewer/headless, disable-hands, record, spark-overlay, tjvr bind/port, existing Manus options, pico calibration directory, pico APK package, pico ROS domain, and pico startup timeout.
- Startup order is ADB/APK, driver, raw preflight, M0, M0 preflight, TJVR bridge, current downstream.
- Shutdown order is downstream, TJVR, M0, driver, and only the supervisor-owned tcp:9999 forward.

- [x] Step 1: Write failing lifecycle tests.

Use temporary fake adb, pixi, preflight, original component scripts, and downstream scripts. Record events and assert the exact sequence adb, driver, raw, m0, m0, bridge, downstream, stop-bridge, stop-m0, stop-driver. Assert that --tjvr-port 15001 and --record reach downstream as vr_manus_sim, while embedded-only options do not. Assert that pico2_hands_sim --resolve-only creates no embedded event.

- [x] Step 2: Run the tests and verify failure.

~~~bash
PYTHONPATH=src/tianji_teleop pixi run python -m pytest tests/test_embedded_pico_launcher.py -q
~~~

Expected result: failure because the supervisor and dispatch are absent.

- [x] Step 3: Implement the supervisor.

Resolve ROOT and the embedded bundle from BASH_SOURCE. Validate the profile, ports, ROS domain range 0..232, timeout, mutually exclusive display options, and disabled-hands Manus options before side effects. Check adb get-state, launch the configured APK with adb shell monkey -p, and establish only tcp:9999. Check the nested install/local_setup.bash. Start the original driver, M0, and TJVR launch through separate setsid children under the nested Pixi manifest. Share ROS_DOMAIN_ID, EXO_REQUESTED_ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY, ROS2CLI_DISABLE_DAEMON, and PICO_TRACKER_CONFIG_DIR with every child.

Run embedded_pico_preflight.py in raw, m0, and bridge modes between stages. Run current scripts/run_session.sh with profile vr_manus_sim and the downstream options. Install EXIT/INT/TERM traps before the first child; stop process groups in reverse order; preserve the downstream return code. Track whether tcp:9999 existed before startup and remove it only if this supervisor created it. Never call adb forward --remove-all.

Patch run_session.sh to preserve original_args before parsing, recognize embedded-only options, include pico_vr_manus_sim in resolve-only choices, dispatch only that profile to the supervisor, and reject embedded-only options on all other profiles. Do not alter the pico2_hands_sim branch or its environment.

- [x] Step 4: Run shell syntax and focused lifecycle tests.

~~~bash
bash -n scripts/run_session.sh scripts/run_embedded_pico_vr_session.sh
PYTHONPATH=src/tianji_teleop pixi run python -m pytest tests/test_embedded_pico_launcher.py -q
~~~

Expected result: syntax check and all lifecycle tests pass.

---

### Task 5: Add the isolated nested build command

**Files:**
- Create: scripts/build_embedded_pico.sh
- Modify: pixi.toml
- Test: tests/test_embedded_pico_bundle.py

**Interfaces:**
- pixi run build-embedded-pico installs and builds only vendor/pico_tracker using its locked manifest.
- The root current project build directory and PICO2 runtime are not touched.

- [x] Step 1: Write failing build-contract tests.

Assert that root pixi.toml contains build-embedded-pico and the script invokes --manifest-path vendor/pico_tracker/pixi.toml and build-pico-bridge. Assert that the script does not read an external PICO_TRACKER_ROOT.

- [x] Step 2: Run the tests and verify failure.

~~~bash
PYTHONPATH=src/tianji_teleop pixi run python -m pytest tests/test_embedded_pico_bundle.py -q
~~~

Expected result: failure because the root task and build script do not exist.

- [x] Step 3: Implement the isolated build command.

The script must resolve its own repository root, verify vendor/pico_tracker/pixi.toml and pixi.lock, run pixi install --manifest-path vendor/pico_tracker/pixi.toml --locked, then run pixi run --manifest-path vendor/pico_tracker/pixi.toml build-pico-bridge (which builds imu_ros2 before pico_bridge). Add only the root task build-embedded-pico. Do not add ROS packages to the root dependencies.

- [x] Step 4: Run the contract tests and build if dependencies are available.

~~~bash
PYTHONPATH=src/tianji_teleop pixi run python -m pytest tests/test_embedded_pico_bundle.py -q
bash -n scripts/build_embedded_pico.sh
pixi run build-embedded-pico
~~~

Expected result: tests and syntax pass; when dependencies are available, vendor/pico_tracker/install/local_setup.bash is created. If dependency download is unavailable, retain the exact error as an environment blocker and continue with non-build tests.

---

### Task 6: Document and protect both input routes

**Files:**
- Modify: README.md
- Modify: docs/real-input-simulation-acceptance.md
- Modify: docs/dual-input-integration-progress.md
- Test: tests/test_dual_session_config.py
- Test: tests/test_dual_session_launcher.py

**Interfaces:**
- The README command for pico_vr_manus_sim starts the embedded upstream and current MuJoCo.
- The existing PICO2 command remains unchanged and explicitly uses the PICO2 route.
- Manual vr_manus_sim remains available for externally supplied TJVR.

- [x] Step 1: Write the isolation regression test.

Assert that the pico2_hands_sim branch in run_session.sh cannot dispatch run_embedded_pico_vr_session.sh, cannot set PICO_TRACKER_CONFIG_DIR, and retains its existing resolver/configuration test results.

- [x] Step 2: Run the focused test and verify failure if dispatch is not scoped.

~~~bash
PYTHONPATH=src/tianji_teleop pixi run python -m pytest tests/test_dual_session_config.py tests/test_dual_session_launcher.py -q
~~~

Expected result: the new assertion fails until dispatch is restricted to pico_vr_manus_sim.

- [x] Step 3: Document the new entry.

Add this command to README.md:

~~~bash
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh \
  --profile pico_vr_manus_sim \
  --viewer \
  --disable-hands
~~~

Explain that the APK is still device-side, calibration defaults to HOME/.config/pico_tracker or PICO_TRACKER_CONFIG_DIR, --disable-hands tests arms without Manus, and removing it requires existing Manus rawviz/SDK arguments. Explain that pico2_hands_sim uses tcp:10002 and is not started by this profile.

- [x] Step 4: Run the regression tests.

~~~bash
PYTHONPATH=src/tianji_teleop pixi run python -m pytest tests/test_dual_session_config.py tests/test_dual_session_launcher.py -q
~~~

Expected result: all tests pass.

---

### Task 7: Full verification and acceptance

**Files:**
- Modify only a file identified by a concrete failing verification.
- Do not stage or modify unrelated pre-existing dirty files.

- [x] Step 1: Run static checks.

~~~bash
git diff --check
bash -n scripts/*.sh vendor/pico_tracker/scripts/*.sh
~~~

- [x] Step 2: Run focused embedded tests.

~~~bash
PYTHONPATH=src/tianji_teleop pixi run python -m pytest \
  tests/test_embedded_pico_bundle.py \
  tests/test_embedded_pico_runtime.py \
  tests/test_embedded_pico_launcher.py \
  tests/test_dual_session_config.py \
  tests/test_dual_session_launcher.py -q
~~~

- [x] Step 3: Run PICO2 and TJVR regressions.

~~~bash
PYTHONPATH=src/tianji_teleop pixi run python -m pytest \
  tests/test_pico_mujoco_pipeline.py \
  tests/test_pico_hand_loop.py \
  tests/test_pico_hand_producer.py \
  tests/test_pico_recording_metadata.py \
  tests/test_reference_tjvr_receiver.py \
  tests/test_spark_coordinated_sim.py -q
~~~

- [x] Step 4: Run profile resolution checks.

~~~bash
pixi run bash scripts/run_session.sh --profile pico2_hands_sim --resolve-only
pixi run bash scripts/run_session.sh --profile vr_manus_sim --disable-hands --resolve-only
pixi run bash scripts/run_session.sh --profile pico_vr_manus_sim --disable-hands --resolve-only
~~~

Expected result: all return 0; only pico_vr_manus_sim reports embedded upstream ownership.

- [x] Step 5: Run the existing full Python suite.

~~~bash
pixi run python -m unittest discover -s tests -p 'test*.py' -v
~~~

Report environment-conditioned skips separately. Do not label skipped device/build tests as passing.

- [x] Step 6: Perform hardware-independent startup acceptance.

Use the fake runtime tests to verify startup order, option forwarding, reverse cleanup, duplicate-start rejection, and PICO2 non-dispatch. Do not claim real-device acceptance until a PICO device and the original APK are connected.

- [x] Step 7: Review the final feature diff.

~~~bash
git status --short
git diff --stat
git diff --check
~~~

Confirm no personal recordings, calibration, APK, private SDK, generated build output, external checkout path, or unrelated existing change is staged.
