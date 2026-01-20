cmake_minimum_required(VERSION 3.18)

project(FluidSimulator LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_EXTENSIONS OFF)

add_library(fluid_core
  core/src/core.cpp
  core/src/init.cpp
)

target_include_directories(fluid_core
  PUBLIC
    core/include
)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  target_link_libraries(fluid_core PUBLIC OpenMP::OpenMP_CXX)
endif()

add_library(fluid_io
  io/src/vtk_writer.cpp
)

target_include_directories(fluid_io
  PUBLIC
    io/include
)

add_library(fluid_cli
  cli/src/cli.cpp
)

target_include_directories(fluid_cli
  PUBLIC
    cli/include
)

add_executable(fluidsim
  app/src/main.cpp
)

target_link_libraries(fluidsim
  PRIVATE
    fluid_core
    fluid_cli
    fluid_io
)

add_library(fluid_cuda
  cuda/src/cuda_stub.cu
)

target_include_directories(fluid_cuda
  PUBLIC
    cuda/include
)

target_link_libraries(fluid_cuda
  PUBLIC
    fluid_core
)

target_link_libraries(fluidsim
  PRIVATE
    fluid_cuda
)
