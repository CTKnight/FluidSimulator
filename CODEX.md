# CODEX.md

This document is for Codex (and other assistants) working on a rewrite of
FluidSimulator. Keep it short, practical, and updated as the rewrite progresses.

## Rewrite goals (must-haves)
1) Correctly implement Position Based Fluids/Particles on both CPU and CUDA with
   parity checks for correctness.
2) Apply high-performance CUDA techniques to scale particle counts.
3) Provide a simple visualization path to inspect particles (headless output
   and/or online view).

## Current state
- Clean-slate rewrite on `master`; legacy implementation retained on `legacy`.
- Repository layout and build system are being reintroduced; update this file as
  paths and tooling stabilize.

## CPU/CUDA parity checklist
- Use identical initial conditions, timesteps, solver iterations, and
  parameters on both paths.
- Keep deterministic ordering for reductions and neighbor processing when
  possible (avoid non-deterministic atomics on critical sums).
- Add a parity harness early (frame dumps + diff tool); track max_abs/RMSE and
  investigate regressions immediately.
- Record parity expectations in this file when introducing new features
  (e.g., collision shapes, vorticity, viscosity).

## CUDA performance guidance (for rewrite)
- Favor SoA layouts for positions/velocities/forces to improve coalescing.
- Use a uniform grid or hashed grid for neighbor search; limit checks to nearby
  cells only.
- Fuse kernels when it reduces global memory traffic; avoid excessive kernel
  launches per substep.
- Minimize host/device synchronization; batch steps, reuse buffers, and use
  pinned memory for any required transfers.
- Be explicit about precision choices (float vs double) and their parity impact.

## Visualization guidance
- Define a simple output format early (CSV/JSON/VTK) to support headless
  captures and validation.
- Provide at least one lightweight viewer (desktop or web) that can load the
  headless output.
- Keep output conversion steps scripted and documented.

## Rewrite workflow expectations
- Make changes in small, testable steps; keep CPU and CUDA in lockstep.
- Update `README.md` and this file when build or run steps change.
- Add minimal validation scripts when new subsystems are introduced
  (collision types, solvers, IO formats).
