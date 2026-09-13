#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"

command -v pixi >/dev/null 2>&1 || {
  printf '%s\n' '错误：找不到 pixi；请先安装 Pixi。' >&2
  exit 2
}
[[ -f "$ROOT/vendor/pico_tracker/pixi.toml" ]] || {
  printf '%s\n' '错误：缺少 vendor/pico_tracker/pixi.toml。' >&2
  exit 2
}
[[ -f "$ROOT/vendor/pico_tracker/pixi.lock" ]] || {
  printf '%s\n' '错误：缺少 vendor/pico_tracker/pixi.lock。' >&2
  exit 2
}

cd -- "$ROOT"
pixi install --manifest-path vendor/pico_tracker/pixi.toml --locked
exec pixi run --manifest-path vendor/pico_tracker/pixi.toml build-pico-bridge
