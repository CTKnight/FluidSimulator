# Fluid Simulator

Position Based Fluids simulation and surface reconstruction for the CS284A
final project.

## Highlights
- Position Based Fluids (PBF) solver with an interactive OpenGL/NanoGUI viewer.
- Optional CUDA path for faster simulation and large particle counts.
- Marching cubes surface extraction for mesh reconstruction.
- Headless mode with particle cache output for offline rendering.

## Project resources
- [Project website](https://ctknight.github.io/FluidSimulator/)
- [Final report (PDF)](https://drive.google.com/file/d/14NBuwOkBv4B0xg0AGVYWsEsgpjXjXg6a/view?usp=sharing)
- [Final video](https://youtu.be/SfTIv-HlWFM)
- [Slides](https://docs.google.com/presentation/d/1VrIeeL3HWLHeoGgKl4LTh8TZWxva0XWIZIpgnCNel8A/edit?usp=sharing)

## Build
Quick start (see `docs/BUILDING.md` for details):
1) Initialize submodules:
   `git submodule update --init --recursive`
2) Configure and build:
   `mkdir build && cd build`
   `cmake ..`
   `cmake --build . --config Release`

CUDA build (optional):
`cmake -DBUILD_CUDA=ON -DCMAKE_CUDA_FLAGS="-arch=sm_60" ..`

Requirements:
- CMake 3.9+
- C++17 toolchain
- OpenGL
- CUDA 10.2 (optional, for `fluidsimCuda`)

## Run
Executables are placed in the repo root after building.

CPU viewer:
`./fluidsim -f scene/fluid.json`

Headless CPU run with particle cache output:
`./fluidsim -n -s 5 -p output/frames -f scene/fluid_large.json`

CUDA run (headless):
`./fluidsimCuda -s 5 -p output/frames -f scene/fluid_large.json`

Common options for `fluidsim`:
- `-f <path>` scene JSON (see `scene/`)
- `-r <path>` project root (must contain `shaders/Default.vert`)
- `-a <int>` sphere latitude segments
- `-o <int>` sphere longitude segments
- `-p <path>` output particle folder
- `-i <path>` input particle folder (replay)
- `-n` disable window (headless)
- `-s <int>` duration in seconds (headless mode)

Options for `fluidsimCuda`:
- `-f <path>` scene JSON
- `-r <path>` project root
- `-p <path>` output particle folder
- `-s <int>` duration in seconds

Note: the CUDA path currently supports plane collisions only.

## Scenes and data
Scene configurations live in `scene/` (for example `scene/fluid.json`,
`scene/fluid_million.json`). Use `-f` to switch between them.

## More docs
- `docs/BUILDING.md` for build notes
- `docs/PROPOSAL.md`, `docs/MILESTONE.md`, `docs/FINAL.md` for project writeups
