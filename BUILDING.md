# Building instruction

## Cmake default command
- Generating solutions
  `cmake -G"Visual Studio 14 2015 Win64" ..`

  add `-DCMAKE_CUDA_FLAGS="-arch=sm_60"` if contains cuda code
- Compiling projects
  `cmake --build . --config Release`
- Installing dependencies
  `cmake --build . --config Release --target install`

## Dependencies

- 