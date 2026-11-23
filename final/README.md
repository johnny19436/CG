# GPU Gems 38 Fluid Sim (CPU vs CUDA)

This project reproduces the core 2D stable-fluids simulation pipeline from GPU Gems Chapter 38 with:

- CPU reference implementation
- CUDA GPU implementation
- CPU-GPU correctness verification
- Performance benchmarks (kernel-only and end-to-end)
- OpenGL real-time visualization with CUDA-OpenGL interop
- Open Slide presentation workspace under `slide/`

## Slides

The `slide/` directory is an Open Slide workspace used for the project presentation.
It authors the deck as React components and can be run locally with:

```bash
cd slide
pnpm install
pnpm dev
```

Then open `http://localhost:5173/` and select `gpu-gems-fluid-sim`.

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## Run correctness check

```bash
./build/fluid_sim verify --width 256 --height 256 --steps 100 --iters 40
```

Optional knobs:

- `--dt 0.1`
- `--dx 1.0`
- `--viscosity 0.00001` (kinematic viscosity; `0` disables velocity diffusion)
- `--diffusion-iters 20` (Jacobi iterations for implicit velocity diffusion, Stam-style)
- `--force-amplitude 4.0`
- `--force-sigma 0.08`

Simulation order per step: advect velocity → diffuse velocity (if viscosity &gt; 0) → add forces → dye → pressure projection.

## Run benchmark (Milestone 4)

This sweeps:

- grid sizes: `128, 256, 512, 1024`
- Jacobi iterations: `20, 40, 80`

```bash
./build/fluid_sim bench --steps 100 --warmup 5 --runs 50
```

It prints:

- `CPU/step (ms)`
- `GPU-kernel/step (ms)`
- `GPU-e2e/step (ms)`
- `Speedup(CPU/kernel)`

## Run visualization

GPU backend (smooth, real-time; CUDA sim + OpenGL PBO interop):

```bash
./build/fluid_sim visualize --backend gpu --width 512 --height 512 --iters 40 --dt 0.1 --steps-per-frame 1 --field dye --vsync 1
```

CPU backend (same physics per step; frame rate drops with grid size and Jacobi iterations):

```bash
./build/fluid_sim visualize --backend cpu --width 512 --height 512 --iters 40 --dt 0.1 --steps-per-frame 1 --field dye --vsync 0
```

Optional `--cpu-min-step-ms N` adds a minimum wall-clock time per simulation batch (useful to exaggerate lag in side-by-side demos; `0` = natural CPU speed only).

Controls:

- `Esc`: quit
- `Space`: pause/resume simulation
- `1`: velocity magnitude
- `2`: pressure
- `3`: divergence
- `4`: dye (smoke-like passive scalar, easiest to interpret)
- Left mouse drag: inject force + dye (interactive stirring)
- Tip: switch to `--field dye` (or press `4`) to clearly see mouse injection

Notes:

- `--backend gpu` runs the solver on the GPU and uploads frames via CUDA–OpenGL PBO interop.
- `--backend cpu` runs the same solver on the CPU and uploads RGBA from host memory each frame.
- For a clearly laggy CPU demo at 512², try `--width 1024` or `--cpu-min-step-ms 80`.
- In headless/SSH environments without an active display, visualization startup may fail at `glfwInit`.

