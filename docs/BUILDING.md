# Building instruction

## Linux/WSL2 (CMake)
- Init submodules:
  `git submodule update --init --recursive`
- Install deps (Ubuntu/WSL2):
  `sudo apt-get update && sudo apt-get install -y cmake build-essential libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libxext-dev libxrender-dev libxfixes-dev libxau-dev libxcb1-dev libxxf86vm-dev libgl1-mesa-dev mesa-common-dev libfreetype6-dev`
- CPU build:
  `cmake -S . -B build`
  `cmake --build build -j`
- CUDA build (optional; set the arch for your GPU, example below is RTX 4090):
  `cmake -S . -B build-cuda -DBUILD_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=89`
  `cmake --build build-cuda -j`

## Windows (Visual Studio)
- Init submodules:
  `git submodule update --init --recursive`
- Configure:
  `cmake -S . -B build -G "Visual Studio 14 2015 Win64"`
- Build:
  `cmake --build build --config Release`

## Dependencies
- CMake 3.9+
- C++17 toolchain (CPU build)
- CUDA toolkit (optional; CUDA 12+/13 requires C++17 for host code)
- OpenGL + X11 dev libraries (Linux/WSL2 viewer)
- Freetype dev headers
