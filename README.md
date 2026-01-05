# Fluid Simulator (Rewrite)

A clean-slate rewrite of a Position Based Fluids/Particles simulator focused on
CPU/CUDA parity, high-performance CUDA kernels, and a simple visualization path.

## Goals
- CPU and CUDA parity for the PBF/PBD solver (deterministic checks).
- High-performance CUDA implementation for large particle counts.
- Simple visualization (headless outputs and/or a lightweight viewer).

## Status
- Rewrite in progress; build/run docs will be added once the new scaffold lands.
- See `CODEX.md` for rewrite guidance and parity expectations.

## Legacy
- The previous implementation and project writeups live on the [`legacy`](https://github.com/CTKnight/FluidSimulator/tree/legacy) branch, see also: legacy [README](https://github.com/CTKnight/FluidSimulator/blob/legacy/README.md).
