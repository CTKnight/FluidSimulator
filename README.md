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

## Docker Build
Image (CUDA base; CPU backend still works):
1) `docker build -t fluidsim .`
2) `docker run --rm fluidsim ./build/fluidsim --backend=cpu`

CUDA run (requires NVIDIA Container Toolkit + compatible driver):
`docker run --rm --gpus all fluidsim ./build/fluidsim --backend=cuda`

Example with scene + output volume:
`docker run --rm --gpus all -v "$PWD/output_docker_cuda:/opt/fluidsim/output_cuda" fluidsim ./build/fluidsim --backend=cuda --scene scene/fluid_million.json --steps-per-sec 120 --fps 30 --duration 4 --enable-scorr --enable-xsph --plane-restitution 0.05 --plane-friction 0.1 --output-dir output_cuda --debug-print`

## Legacy
- The previous implementation (CS 284A Spring 2020 final project) moved to the [`legacy`](https://github.com/CTKnight/FluidSimulator/tree/legacy) branch, see also: legacy [README](https://github.com/CTKnight/FluidSimulator/blob/legacy/README.md).
