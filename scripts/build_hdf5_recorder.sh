#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if ! command -v h5c++ >/dev/null; then
  echo 'Missing h5c++: install libhdf5-dev and g++ before building.' >&2
  exit 1
fi
mkdir -p "$ROOT/build/hdf5_recorder"
cd "$ROOT/build/hdf5_recorder"
h5c++ -std=c++17 -O3 -Wall -Wextra -Werror \
  "$ROOT/native/hdf5_recorder/main.cpp" -o tianji_hdf5_recorder.tmp
mv tianji_hdf5_recorder.tmp tianji_hdf5_recorder
echo "Built $ROOT/build/hdf5_recorder/tianji_hdf5_recorder"
