#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

SCENE="${1:-scene/fluid.json}"
DURATION="${DURATION:-1}"
CPU_BIN="${CPU_BIN:-$ROOT/build/fluidsim}"
CUDA_BIN="${CUDA_BIN:-$ROOT/build-cuda/fluidsimCuda}"
EPS="${EPS:-1e-5}"

if [[ ! -x "$CPU_BIN" ]]; then
  echo "Missing CPU binary: $CPU_BIN"
  echo "Build it with: cmake --build build -j"
  exit 1
fi
if [[ ! -x "$CUDA_BIN" ]]; then
  echo "Missing CUDA binary: $CUDA_BIN"
  echo "Build it with: cmake --build build-cuda -j"
  exit 1
fi

OUT_BASE="${OUT_BASE:-$ROOT/output/compare-$(date +%Y%m%d-%H%M%S)}"
CPU_OUT="$OUT_BASE/cpu"
CUDA_OUT="$OUT_BASE/cuda"
CPU_LOG="$OUT_BASE/cpu.log"
CUDA_LOG="$OUT_BASE/cuda.log"

mkdir -p "$CPU_OUT" "$CUDA_OUT"

echo "Running CPU -> $CPU_OUT"
"$CPU_BIN" -n -s "$DURATION" -p "$CPU_OUT" -f "$SCENE" >"$CPU_LOG" 2>&1

echo "Running CUDA -> $CUDA_OUT"
if ! "$CUDA_BIN" -s "$DURATION" -p "$CUDA_OUT" -f "$SCENE" >"$CUDA_LOG" 2>&1; then
  echo "CUDA run failed. See: $CUDA_LOG"
  grep -m1 -E "bad_alloc|cudaError|error" "$CUDA_LOG" || true
  exit 1
fi

PYTHON_BIN="${PYTHON_BIN:-python3}"
if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
  echo "Python not found. Set PYTHON_BIN or install python3 to compare outputs."
  exit 1
fi

OUT_BASE="$OUT_BASE" EPS="$EPS" "$PYTHON_BIN" - <<'PY'
import glob
import math
import os
import sys

out_base = os.environ.get("OUT_BASE")
if not out_base:
    print("OUT_BASE not set")
    sys.exit(1)

cpu_dir = os.path.join(out_base, "cpu")
cuda_dir = os.path.join(out_base, "cuda")
eps = float(os.environ.get("EPS", "1e-5"))

def read_points(path):
    with open(path, "r", encoding="ascii", errors="ignore") as f:
        tokens = f.read().split()
    try:
        idx = tokens.index("POINTS")
    except ValueError:
        raise RuntimeError(f"POINTS not found in {path}")
    count = int(tokens[idx + 1])
    data = list(map(float, tokens[idx + 3:]))
    if len(data) != count * 3:
        raise RuntimeError(f"Expected {count*3} floats, got {len(data)} in {path}")
    return data

cpu_files = sorted(glob.glob(os.path.join(cpu_dir, "fluid*.vtp")))
cuda_files = sorted(glob.glob(os.path.join(cuda_dir, "fluid*.vtp")))

if not cpu_files:
    print(f"No CPU output files in {cpu_dir}")
    sys.exit(1)
if not cuda_files:
    print(f"No CUDA output files in {cuda_dir}")
    sys.exit(1)
if len(cpu_files) != len(cuda_files):
    print(f"Frame count mismatch: CPU {len(cpu_files)} vs CUDA {len(cuda_files)}")
    sys.exit(1)

overall_max = 0.0
overall_sum_sq = 0.0
overall_count = 0
worst_frame = None

for cpu_path, cuda_path in zip(cpu_files, cuda_files):
    cpu = read_points(cpu_path)
    cuda = read_points(cuda_path)
    if len(cpu) != len(cuda):
        print(f"Point count mismatch: {cpu_path} ({len(cpu)}) vs {cuda_path} ({len(cuda)})")
        sys.exit(1)
    max_abs = 0.0
    sum_sq = 0.0
    for a, b in zip(cpu, cuda):
        d = abs(a - b)
        if d > max_abs:
            max_abs = d
        sum_sq += d * d
    rmse = math.sqrt(sum_sq / len(cpu))
    overall_sum_sq += sum_sq
    overall_count += len(cpu)
    if max_abs > overall_max:
        overall_max = max_abs
        worst_frame = os.path.basename(cpu_path)
    status = "OK" if max_abs <= eps else "DIFF"
    print(f"{os.path.basename(cpu_path)}: max_abs={max_abs:.6g} rmse={rmse:.6g} {status}")

overall_rmse = math.sqrt(overall_sum_sq / overall_count)
print(f"Overall: max_abs={overall_max:.6g} rmse={overall_rmse:.6g} worst_frame={worst_frame}")
PY

echo "Done. Outputs: $OUT_BASE"
