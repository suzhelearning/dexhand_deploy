# SPARK native port

Source: TJ_arm_control `c022b17789e9b81141915662b3bb802f4c20a396`.
The mechanically migrated files and their reversible namespace-only changes
are recorded in `source_manifest.json`.

Build this directory independently with `tools/spark_native/pixi.toml`.
It intentionally does not link into the existing Pinocchio backend: the
reference arm dependency is Pinocchio 3.9.0, while the official hand uses 4.0.0.

Full configuration/model/TCP provenance, independent original Viewer oracle,
recorded-trace results, reproducible commands and remaining integration gates:
[SPARK porting report](../../../../../docs/spark-porting.md).

The worker is simulation-only and has no device/network outputs. Deterministic
test mode relaxes wall-clock solver budgets only and is not real-time evidence.
IPC rejects partial lines, future receive times and nonconsecutive ticks before
advancing control state; no implicit restart is performed by the Python client.
