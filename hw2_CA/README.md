# Computer Animation and Special Effects HW2

## Build on Microsoft Windows with Visual Studio 2026

## Build on other platforms and/or compilers

### :warning: **This method is not well-tested, so it may not work properly.**

### :warning: **Thus, you are expected to solve buggy or trivial problems yourself.**

### Some tested platforms (cmake 3.20):

- Ubuntu 24.04 with GCC 10.2.0

### Prerequisite

- [Cmake](https://cmake.org) (version >= 3.16)
- Compiler (e.g. GCC)

### Instruction

- Run:
```bash
cmake -S . -B build
cmake --build build --config Release --parallel 8
```
- Executable will be in ./bin

### If you are building on Linux, you may need one of these dependencies, usually `xorg-dev`

- `libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev` (For X11)
- `libwayland-dev wayland-protocols extra-cmake-modules libxkbcommon-dev` (For Wayland)
- `libosmesa6-dev` (For OSMesa)
