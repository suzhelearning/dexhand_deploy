# Review compatibility fixes

Scope approved in conversation: repair the three review findings without
changing IK, mapping, smoothing parameters or the explicitly selected PICO
simulation workflow. Sequential work; no subagents, commits or hardware runs.

## Changes

1. Recognize the Manus target configuration and resolve it to canonical source
   identity `hand_tracking_target`, retaining the Manus configuration path.
2. Clear inherited `TIANJI_JOINT_TRAJECTORY_PROCESSOR`,
   `TIANJI_COMMAND_STEP_CLIPPING`, and `TIANJI_ARM_TARGET_PROCESSOR` at session
   launch. Explicit supported CLI arguments are applied afterward. Component
   entry points retain their environment interfaces.
3. Write boolean `target/arm/{side}/tracking_valid` in new schema 1.1 files.
   Validate its shape, dtype and row count when present. Older 1.0/1.1 files
   lacking it read as tracking-valid. Schema 1.0 layout is unchanged and rejects
   invalid-tracking targets before any row is appended.

## Verification

- Added failing tests for Manus launch, inherited overrides and HDF5 round-trip
  before implementation; added backward compatibility and malformed-data tests.
- Use isolated mock bundles/processes for launcher tests; never connect hardware.
- Exercise the user's explicit PICO passthrough/URDF/overlay CLI in the mock
  launcher and run existing tracking-hold and mapper tests.
- Missing `tianji_world_output` remains an acknowledged, out-of-scope dependency.

The HDF5 change preserves hold semantics in recordings; this repair does not
implement a new hand-tracking target replay pipeline.
