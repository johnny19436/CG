cmake_minimum_required(VERSION 3.24)
project(fluid_gpu_gems38 LANGUAGES C CXX CUDA)

include(FetchContent)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_X11 ON CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)

find_package(OpenGL REQUIRED)

FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
)
FetchContent_MakeAvailable(glfw)

add_executable(fluid_sim
    src/main.cpp
    src/cpu_sim.cpp
    src/gpu_sim.cu
    src/visualize_gl.cu
    src/visualize_gl_cpu.cpp
)

target_include_directories(fluid_sim PRIVATE src)

set_target_properties(fluid_sim PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_ARCHITECTURES 86
)

target_link_libraries(fluid_sim PRIVATE
    OpenGL::GL
    glfw
)

