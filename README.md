# Fluid Simulator (Rewrite WIP)

A Position Based Fluids/Particles simulator focused on
CPU/CUDA parity, high-performance CUDA kernels, and a simple visualization path.

## Goals
- CPU and CUDA parity for the PBF/PBD solver.
- High-performance CUDA implementation for large particle counts.
- Simple visualization (headless outputs or on-the-fly viewer).

## Status
- Rewrite in progress; minimal build/run docs are below.
- See `CODEX.md` for rewrite guidance and parity expectations.

## Build
Single build (CPU + optional CUDA backend):
1) `cmake -S . -B build -DBUILD_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=89`
2) `cmake --build build -j`

If you do not have CUDA, omit `-DBUILD_CUDA=ON` and `CMAKE_CUDA_ARCHITECTURES`.

Run (CPU): `./build/fluidsim --backend=cpu`
Run (CUDA): `./build/fluidsim --backend=cuda` (requires CUDA build)

## Legacy
- The previous implementation (CS 284A Spring 2020 final project) moved to the [`legacy`](https://github.com/CTKnight/FluidSimulator/tree/legacy) branch, see also: legacy [README](https://github.com/CTKnight/FluidSimulator/blob/legacy/README.md).
