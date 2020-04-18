# Building instruction

## Projects Cmake default command
- Git clone pull submodule
  `git submodule update --init --recursive`
- Make build dir
  `mkdir build && cd build`
- Generating solutions
  - without CUDA version
    `cmake -G"Visual Studio 14 2015 Win64" ..`
  - with CUDA version
    `cmake -G"Visual Studio 14 2015 Win64" -DBUILD_CUDA=ON -DCMAKE_CUDA_FLAGS="-arch=sm_60" ..`
- Compiling projects
  `cmake --build . --config Release`

## Dependencies
- CUDA 10.2
- ext/cuNSearch
- ext/CompactNSearch