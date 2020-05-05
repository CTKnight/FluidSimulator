cmake_minimum_required(VERSION 3.9)

# Cloth simulation source
set(FLUIDSIM_VIEWER_SOURCE
    # Cloth simulation objects
    fluid.cpp
    clothMesh.cpp

    # Collision objects
    collision/sphere.cpp
    collision/plane.cpp

    # Application
    main.cpp
    clothSimulator.cpp

    # Miscellaneous
    # png.cpp
    misc/sphere_drawing.cpp
    misc/file_utils.cpp

    # Camera
    camera.cpp

    # Renderer
    renderer.cpp
)

set(FLUIDSIM_CUDA_SOURCE
    # Cloth simulation objects
    fluid.cpp

    # Collision objects
    collision/sphere.cpp
    collision/plane.cpp

    # Application
    cuda/cudaMain.cu

    misc/sphere_drawing.cpp
    misc/file_utils.cpp

    cuda/cudaFluid.cu
    cuda/cudaFluid.cuh
)

# Windows-only sources
if(WIN32)
list(APPEND FLUIDSIM_VIEWER_SOURCE
    # For get-opt
    misc/getopt.c
)

list(APPEND FLUIDSIM_CUDA_SOURCE
    # For get-opt
    misc/getopt.c
)
endif(WIN32)

#-------------------------------------------------------------------------------
# Embed resources
#-------------------------------------------------------------------------------

# embed_resources(CLOTHSIM_SHADERS
#   shaders/vert.glsl
#   shaders/normalFrag.glsl
#   shaders/wireframeFrag.glsl
# )

#-------------------------------------------------------------------------------
# Set definitions
#-------------------------------------------------------------------------------
add_definitions(${NANOGUI_EXTRA_DEFS} ${CompactNSearch_EXTRA_DEFS})

#-------------------------------------------------------------------------------
# Set include directories
#-------------------------------------------------------------------------------
include_directories(
  ${CGL_INCLUDE_DIRS}
  ${FREETYPE_INCLUDE_DIRS}
  ${NANOGUI_EXTRA_INCS}
  ${CompactNSearch_EXTRA_INCS}
)

#-------------------------------------------------------------------------------
# Set link directories
#-------------------------------------------------------------------------------
link_directories(
  ${CGL_LIBRARY_DIRS}
  ${FREETYPE_LIBRARY_DIRS}
)

#-------------------------------------------------------------------------------
# Add executable
#-------------------------------------------------------------------------------
add_definitions(-D_USE_MATH_DEFINES)

add_executable(fluidsim ${FLUIDSIM_VIEWER_SOURCE})
target_link_libraries(fluidsim
    CGL ${CGL_LIBRARIES}
    nanogui ${NANOGUI_EXTRA_LIBS}
    ${FREETYPE_LIBRARIES}
    ${CMAKE_THREADS_INIT}
    CompactNSearch
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(fluidsim
        OpenMP::OpenMP_CXX
    )
endif(OpenMP_CXX_FOUND)

if (BUILD_CUDA)
    enable_language(CUDA)
    include(CheckLanguage)
    check_language(CUDA)
    include_directories(
        ${CUDA_INCLUDE_DIRS}
    )
    target_link_libraries(fluidsim
        cuNSearch
    )
    # Cloth simulation source
    add_executable(fluidsimCuda ${FLUIDSIM_CUDA_SOURCE})
    set_target_properties(fluidsimCuda PROPERTIES
                            CUDA_SEPERABLE_COMPILATION ON)
    target_link_libraries(fluidsimCuda
        CGL ${CGL_LIBRARIES}
        ${CMAKE_THREADS_INIT}
        CompactNSearch
        cuNSearch
        ${CUDA_LIBRARIES}
    )
endif (BUILD_CUDA)

#-------------------------------------------------------------------------------
# Platform-specific configurations for target
#-------------------------------------------------------------------------------
if(APPLE)
  set_property( TARGET fluidsim APPEND_STRING PROPERTY COMPILE_FLAGS
                "-Wno-deprecated-declarations -Wno-c++11-extensions")
endif(APPLE)

# Put executable in build directory root
set(EXECUTABLE_OUTPUT_PATH ..)

# Install to project root
install(TARGETS fluidsim DESTINATION ${FluidSim_SOURCE_DIR})
