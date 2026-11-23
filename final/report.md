Project Overview

Reproduce the core fluid simulation pipeline from GPU Gems Chapter 38

Implement both a CPU reference version and a CUDA GPU version

Compare correctness and performance between CPU and GPU
---

GPU Gems Chapter 38

Simulates 2D incompressible fluid on a regular grid

Uses the Stable Fluids method by Jos Stam

Core pipeline: advection, diffusion, external force, and projection
---

Simulation Framework

This project uses an Eulerian grid-based framework

Velocity, pressure, divergence, and dye are stored at fixed grid cells

Advection is implemented with semi-Lagrangian backtracing
---

Eulerian Grid Representation

Each fluid cell stores simulation quantities

CPU stores fields in std::vector<float>

GPU stores fields in CUDA device memory

2D indexing is mapped by idx(x, y, width) = y * width + x
---

CPU vs GPU Mapping

CPU updates cells with nested loops

GPU updates cells with CUDA kernels

Each CUDA thread computes one fluid cell

The algorithm is the same, but the execution model is parallel on GPU
---

Main Simulation Fields

Velocity field: vx and vy

Pressure field: pressure

Divergence field: divergence

Dye field: passive tracer for visualization
---

One Simulation Step

Advect velocity

Diffuse velocity

Apply external force

Advect dye

Project velocity to reduce divergence
---

Advection

Advection transports quantities through the velocity field

The velocity field advects itself

Dye is also advected by the velocity field

Implemented with backward tracing and bilinear sampling
---

Velocity Advection

For each cell, trace backward using the current velocity

Sample the previous velocity at the traced position

Write the result into the next velocity buffer
---

Diffusion

Diffusion models viscosity

Viscosity smooths the velocity field over time

Higher viscosity makes the fluid look thicker and slower
---

Implicit Diffusion

The project solves diffusion using Jacobi iteration

vx and vy are diffused separately

Diffusion is enabled when viscosity is greater than zero
---

Diffusion Parameters

--viscosity controls the strength of diffusion

--diffusion-iters controls Jacobi iterations for diffusion

More iterations improve accuracy but increase runtime
---

External Forces

The simulation applies a fixed Gaussian force near the center

The force is strongest at the center and smoothly decreases outward

Mouse interaction can also inject local velocity impulses
---

Mouse Interaction

Mouse dragging adds local force to the velocity field

Mouse input also adds dye for visualization

The dye is only a passive tracer and does not affect pressure or velocity
---

Dye Field

Dye represents passive tracer concentration

It helps visualize the motion of the fluid

Dye is not fluid density and does not create pressure
---

Projection

Projection is the key step for incompressible flow

It transforms the velocity field into a nearly divergence-free field

This keeps the fluid from artificially expanding or compressing
---

Why Projection Is Needed

Advection, diffusion, and forces can create divergence

Divergence means local flow is expanding or contracting

Incompressible fluid requires divergence to be close to zero
---

Projection Pipeline

Compute divergence from the velocity field

Solve pressure using Jacobi iteration

Subtract the pressure gradient from velocity
---

Divergence Computation

Divergence measures local inflow and outflow

The project uses central difference approximation

div = 0.5 / dx * (vx_right - vx_left + vy_up - vy_down)
---

Pressure Solve

Pressure is computed from the divergence field

The project solves a Poisson pressure equation

Jacobi iteration is used for both CPU and GPU versions
---

Pressure Jacobi Iteration

Each cell updates pressure from its four neighbors

More pressure iterations usually reduce divergence more

--iters controls the number of pressure Jacobi iterations
---

Subtract Pressure Gradient

After pressure is solved, its gradient is subtracted from velocity

u = w - grad(p)

This produces a velocity field closer to incompressible flow
---

Boundary Conditions

Velocity boundary conditions prevent flow through solid walls

Pressure boundary values copy nearby interior pressure

Boundary handling is applied after major simulation steps
---

Visualization

OpenGL is used for real-time visualization

CUDA-OpenGL interop is used for GPU rendering

The simulation can display velocity, pressure, divergence, and dye
---

Visualization Controls

Key 1 displays velocity magnitude

Key 2 displays pressure

Key 3 displays divergence

Key 4 displays dye
---

Color Interpretation

Velocity uses a heat map for speed magnitude

Pressure and divergence use a diverging color map

Blue means negative, green means near zero, warm colors mean positive
---

Example Command

./build/fluid_sim visualize --backend gpu --width 512 --height 512 --iters 40 --viscosity 0.00001 --diffusion-iters 20 --dt 0.1 --steps-per-frame 1 --field dye --vsync 1
---

Command Parameters

--backend gpu selects the CUDA implementation

--width and --height set the simulation grid size

--dt sets the simulation time step

--steps-per-frame controls simulation steps per rendered frame
---

Performance Evaluation

CPU time is measured with standard timing utilities

GPU time is measured with CUDA events

The benchmark reports CPU time, GPU kernel time, and end-to-end GPU time
---

Correctness Verification

The CPU version is used as the reference

The GPU version starts from the same initial state

The outputs are compared using numerical error metrics
---

Project Limitations

The simulation is 2D only

The fluid is assumed to be incompressible

Dye is passive and does not affect the fluid

No buoyancy, temperature, obstacles, or free-surface flow are implemented
---

Conclusion

The project reproduces the main Stable Fluids pipeline from GPU Gems Chapter 38

The original texture and fragment shader approach is replaced with CUDA kernels

GPU parallelism is effective because each grid cell can be updated independently within each step
