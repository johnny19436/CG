#include "gpu_sim.h"

#include "cpu_sim.h"

#include <cuda_runtime.h>

#include <algorithm>
#include <cstdio>
#include <stdexcept>
#include <utility>

namespace {

void check_cuda(cudaError_t err, const char* msg) {
    if (err != cudaSuccess) {
        throw std::runtime_error(std::string(msg) + ": " + cudaGetErrorString(err));
    }
}

__device__ float clampf_dev(float v, float lo, float hi) {
    return fminf(hi, fmaxf(lo, v));
}

__device__ float sample_bilinear_dev(const float* field, int w, int h, float x, float y) {
    x = clampf_dev(x, 1.0f, static_cast<float>(w - 2));
    y = clampf_dev(y, 1.0f, static_cast<float>(h - 2));

    const int x0 = static_cast<int>(floorf(x));
    const int y0 = static_cast<int>(floorf(y));
    const int x1 = min(x0 + 1, w - 2);
    const int y1 = min(y0 + 1, h - 2);
    const float tx = x - static_cast<float>(x0);
    const float ty = y - static_cast<float>(y0);

    const float v00 = field[idx(x0, y0, w)];
    const float v10 = field[idx(x1, y0, w)];
    const float v01 = field[idx(x0, y1, w)];
    const float v11 = field[idx(x1, y1, w)];

    const float a = v00 + tx * (v10 - v00);
    const float b = v01 + tx * (v11 - v01);
    return a + ty * (b - a);
}

__global__ void advect_velocity_kernel(const float* vx, const float* vy, float* vx_out, float* vy_out, int w, int h, float dt, float dx) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x <= 0 || x >= w - 1 || y <= 0 || y >= h - 1) {
        return;
    }
    const int id = idx(x, y, w);
    const float back_x = static_cast<float>(x) - dt * vx[id] / dx;
    const float back_y = static_cast<float>(y) - dt * vy[id] / dx;
    vx_out[id] = sample_bilinear_dev(vx, w, h, back_x, back_y);
    vy_out[id] = sample_bilinear_dev(vy, w, h, back_x, back_y);
}

__global__ void add_force_kernel(float* vx, int w, int h, float dt, float amp, float sigma) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x <= 0 || x >= w - 1 || y <= 0 || y >= h - 1) {
        return;
    }
    const float cx = 0.5f * static_cast<float>(w - 1);
    const float cy = 0.5f * static_cast<float>(h - 1);
    const float sigma_cells = sigma * static_cast<float>(min(w, h));
    const float inv_two_sigma2 = 1.0f / (2.0f * sigma_cells * sigma_cells + 1e-8f);
    const float dx_c = static_cast<float>(x) - cx;
    const float dy_c = static_cast<float>(y) - cy;
    const float g = expf(-(dx_c * dx_c + dy_c * dy_c) * inv_two_sigma2);
    vx[idx(x, y, w)] += dt * amp * g;
}

__global__ void advect_dye_kernel(const float* dye, const float* vx, const float* vy, float* dye_out, int w, int h, float dt, float dx) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x <= 0 || x >= w - 1 || y <= 0 || y >= h - 1) {
        return;
    }
    const int id = idx(x, y, w);
    const float back_x = static_cast<float>(x) - dt * vx[id] / dx;
    const float back_y = static_cast<float>(y) - dt * vy[id] / dx;
    dye_out[id] = sample_bilinear_dev(dye, w, h, back_x, back_y);
}

__global__ void add_dye_source_kernel(float* dye, int w, int h, float dt, float sigma) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x <= 0 || x >= w - 1 || y <= 0 || y >= h - 1) {
        return;
    }
    const float cx = 0.5f * static_cast<float>(w - 1);
    const float cy = 0.5f * static_cast<float>(h - 1);
    const float sigma_cells = sigma * static_cast<float>(min(w, h));
    const float inv_two_sigma2 = 1.0f / (2.0f * sigma_cells * sigma_cells + 1e-8f);
    const float dx_c = static_cast<float>(x) - cx;
    const float dy_c = static_cast<float>(y) - cy;
    const float src = expf(-(dx_c * dx_c + dy_c * dy_c) * inv_two_sigma2);
    const int id = idx(x, y, w);
    dye[id] = fminf(1.0f, fmaxf(0.0f, 0.995f * dye[id] + 0.15f * dt * src));
}

__global__ void mouse_interaction_kernel(float* vx, float* vy, float* dye, int w, int h, float cx, float cy, float fx, float fy, float radius,
                                         float force_scale, float dye_scale) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x <= 0 || x >= w - 1 || y <= 0 || y >= h - 1) {
        return;
    }
    const int id = idx(x, y, w);
    const float dx_c = static_cast<float>(x) - cx;
    const float dy_c = static_cast<float>(y) - cy;
    const float sigma2 = radius * radius + 1e-8f;
    const float wgt = expf(-(dx_c * dx_c + dy_c * dy_c) / (2.0f * sigma2));
    vx[id] += force_scale * fx * wgt;
    vy[id] += force_scale * fy * wgt;
    dye[id] = fminf(1.0f, fmaxf(0.0f, dye[id] + dye_scale * wgt));
}

__global__ void divergence_kernel(const float* vx, const float* vy, float* div, int w, int h, float dx) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x <= 0 || x >= w - 1 || y <= 0 || y >= h - 1) {
        return;
    }
    div[idx(x, y, w)] = 0.5f / dx *
                        (vx[idx(x + 1, y, w)] - vx[idx(x - 1, y, w)] + vy[idx(x, y + 1, w)] - vy[idx(x, y - 1, w)]);
}

__global__ void jacobi_diffuse_kernel(const float* x0, const float* x, float* x_out, int w, int h, float alpha, float inv_beta) {
    const int x_i = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x_i <= 0 || x_i >= w - 1 || y <= 0 || y >= h - 1) {
        return;
    }
    const int id = idx(x_i, y, w);
    const float sum_n = x[idx(x_i + 1, y, w)] + x[idx(x_i - 1, y, w)] + x[idx(x_i, y + 1, w)] + x[idx(x_i, y - 1, w)];
    x_out[id] = (x0[id] + alpha * sum_n) * inv_beta;
}

__global__ void jacobi_pressure_kernel(const float* p, const float* div, float* p_out, int w, int h, float alpha) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x <= 0 || x >= w - 1 || y <= 0 || y >= h - 1) {
        return;
    }
    p_out[idx(x, y, w)] = (p[idx(x + 1, y, w)] + p[idx(x - 1, y, w)] + p[idx(x, y + 1, w)] + p[idx(x, y - 1, w)] -
                           alpha * div[idx(x, y, w)]) *
                          0.25f;
}

__global__ void subtract_gradient_kernel(float* vx, float* vy, const float* p, int w, int h, float dx) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x <= 0 || x >= w - 1 || y <= 0 || y >= h - 1) {
        return;
    }
    vx[idx(x, y, w)] -= 0.5f / dx * (p[idx(x + 1, y, w)] - p[idx(x - 1, y, w)]);
    vy[idx(x, y, w)] -= 0.5f / dx * (p[idx(x, y + 1, w)] - p[idx(x, y - 1, w)]);
}

__global__ void set_zero_kernel(float* a, int n) {
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) {
        a[i] = 0.0f;
    }
}

__global__ void boundary_velocity_lr_kernel(float* vx, float* vy, int w, int h) {
    const int y = blockIdx.x * blockDim.x + threadIdx.x;
    if (y <= 0 || y >= h - 1) {
        return;
    }
    vx[idx(0, y, w)] = -vx[idx(1, y, w)];
    vx[idx(w - 1, y, w)] = -vx[idx(w - 2, y, w)];
    vy[idx(0, y, w)] = vy[idx(1, y, w)];
    vy[idx(w - 1, y, w)] = vy[idx(w - 2, y, w)];
}

__global__ void boundary_velocity_tb_kernel(float* vx, float* vy, int w, int h) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    if (x <= 0 || x >= w - 1) {
        return;
    }
    vx[idx(x, 0, w)] = vx[idx(x, 1, w)];
    vx[idx(x, h - 1, w)] = vx[idx(x, h - 2, w)];
    vy[idx(x, 0, w)] = -vy[idx(x, 1, w)];
    vy[idx(x, h - 1, w)] = -vy[idx(x, h - 2, w)];
}

__global__ void boundary_pressure_lr_kernel(float* p, int w, int h) {
    const int y = blockIdx.x * blockDim.x + threadIdx.x;
    if (y <= 0 || y >= h - 1) {
        return;
    }
    p[idx(0, y, w)] = p[idx(1, y, w)];
    p[idx(w - 1, y, w)] = p[idx(w - 2, y, w)];
}

__global__ void boundary_pressure_tb_kernel(float* p, int w, int h) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    if (x <= 0 || x >= w - 1) {
        return;
    }
    p[idx(x, 0, w)] = p[idx(x, 1, w)];
    p[idx(x, h - 1, w)] = p[idx(x, h - 2, w)];
}

__global__ void boundary_corners_vec_kernel(float* a, int w, int h) {
    a[idx(0, 0, w)] = 0.5f * (a[idx(1, 0, w)] + a[idx(0, 1, w)]);
    a[idx(w - 1, 0, w)] = 0.5f * (a[idx(w - 2, 0, w)] + a[idx(w - 1, 1, w)]);
    a[idx(0, h - 1, w)] = 0.5f * (a[idx(1, h - 1, w)] + a[idx(0, h - 2, w)]);
    a[idx(w - 1, h - 1, w)] = 0.5f * (a[idx(w - 2, h - 1, w)] + a[idx(w - 1, h - 2, w)]);
}

__global__ void boundary_vx_lr_kernel(float* vx, int w, int h) {
    const int y = blockIdx.x * blockDim.x + threadIdx.x;
    if (y <= 0 || y >= h - 1) {
        return;
    }
    vx[idx(0, y, w)] = -vx[idx(1, y, w)];
    vx[idx(w - 1, y, w)] = -vx[idx(w - 2, y, w)];
}

__global__ void boundary_vx_tb_kernel(float* vx, int w, int h) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    if (x <= 0 || x >= w - 1) {
        return;
    }
    vx[idx(x, 0, w)] = vx[idx(x, 1, w)];
    vx[idx(x, h - 1, w)] = vx[idx(x, h - 2, w)];
}

__global__ void boundary_vy_lr_kernel(float* vy, int w, int h) {
    const int y = blockIdx.x * blockDim.x + threadIdx.x;
    if (y <= 0 || y >= h - 1) {
        return;
    }
    vy[idx(0, y, w)] = vy[idx(1, y, w)];
    vy[idx(w - 1, y, w)] = vy[idx(w - 2, y, w)];
}

__global__ void boundary_vy_tb_kernel(float* vy, int w, int h) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    if (x <= 0 || x >= w - 1) {
        return;
    }
    vy[idx(x, 0, w)] = -vy[idx(x, 1, w)];
    vy[idx(x, h - 1, w)] = -vy[idx(x, h - 2, w)];
}

void apply_boundary_vx_gpu(float* vx, int w, int h) {
    const int t = 256;
    boundary_vx_lr_kernel<<<(h + t - 1) / t, t>>>(vx, w, h);
    boundary_vx_tb_kernel<<<(w + t - 1) / t, t>>>(vx, w, h);
    boundary_corners_vec_kernel<<<1, 1>>>(vx, w, h);
    check_cuda(cudaGetLastError(), "apply_boundary_vx_gpu");
}

void apply_boundary_vy_gpu(float* vy, int w, int h) {
    const int t = 256;
    boundary_vy_lr_kernel<<<(h + t - 1) / t, t>>>(vy, w, h);
    boundary_vy_tb_kernel<<<(w + t - 1) / t, t>>>(vy, w, h);
    boundary_corners_vec_kernel<<<1, 1>>>(vy, w, h);
    check_cuda(cudaGetLastError(), "apply_boundary_vy_gpu");
}

void diffuse_velocity_component_gpu(float* x, float* x0, float* scratch, int w, int h, float alpha, float inv_beta, int iters,
                                    bool component_is_vx) {
    dim3 block(16, 16);
    dim3 grid((w + block.x - 1) / block.x, (h + block.y - 1) / block.y);
    const std::size_t bytes = static_cast<std::size_t>(w) * h * sizeof(float);
    check_cuda(cudaMemcpy(x0, x, bytes, cudaMemcpyDeviceToDevice), "copy diffuse x0");

    for (int k = 0; k < iters; ++k) {
        jacobi_diffuse_kernel<<<grid, block>>>(x0, x, scratch, w, h, alpha, inv_beta);
        check_cuda(cudaGetLastError(), "jacobi_diffuse_kernel launch");
        check_cuda(cudaMemcpy(x, scratch, bytes, cudaMemcpyDeviceToDevice), "copy diffuse result");
        if (component_is_vx) {
            apply_boundary_vx_gpu(x, w, h);
        } else {
            apply_boundary_vy_gpu(x, w, h);
        }
    }
}

void diffuse_velocity_gpu(float* vx, float* vy, float* vx_next, float* vy_next, int w, int h, float alpha, float inv_beta, int iters) {
    diffuse_velocity_component_gpu(vx, vx_next, vy_next, w, h, alpha, inv_beta, iters, true);
    diffuse_velocity_component_gpu(vy, vy_next, vx_next, w, h, alpha, inv_beta, iters, false);
}

void apply_boundary_velocity_gpu(float* vx, float* vy, int w, int h) {
    const int t = 256;
    boundary_velocity_lr_kernel<<<(h + t - 1) / t, t>>>(vx, vy, w, h);
    boundary_velocity_tb_kernel<<<(w + t - 1) / t, t>>>(vx, vy, w, h);
    boundary_corners_vec_kernel<<<1, 1>>>(vx, w, h);
    boundary_corners_vec_kernel<<<1, 1>>>(vy, w, h);
}

void apply_boundary_pressure_gpu(float* p, int w, int h) {
    const int t = 256;
    boundary_pressure_lr_kernel<<<(h + t - 1) / t, t>>>(p, w, h);
    boundary_pressure_tb_kernel<<<(w + t - 1) / t, t>>>(p, w, h);
    boundary_corners_vec_kernel<<<1, 1>>>(p, w, h);
}

}  // namespace

GpuFluid::GpuFluid(int width, int height)
    : w_(width), h_(height), n_(width * height), d_vx_(nullptr), d_vy_(nullptr), d_vx_next_(nullptr), d_vy_next_(nullptr), d_p_(nullptr),
      d_p_next_(nullptr), d_div_(nullptr), d_dye_(nullptr), d_dye_next_(nullptr) {
    check_cuda(cudaMalloc(&d_vx_, sizeof(float) * n_), "cudaMalloc d_vx");
    check_cuda(cudaMalloc(&d_vy_, sizeof(float) * n_), "cudaMalloc d_vy");
    check_cuda(cudaMalloc(&d_vx_next_, sizeof(float) * n_), "cudaMalloc d_vx_next");
    check_cuda(cudaMalloc(&d_vy_next_, sizeof(float) * n_), "cudaMalloc d_vy_next");
    check_cuda(cudaMalloc(&d_p_, sizeof(float) * n_), "cudaMalloc d_p");
    check_cuda(cudaMalloc(&d_p_next_, sizeof(float) * n_), "cudaMalloc d_p_next");
    check_cuda(cudaMalloc(&d_div_, sizeof(float) * n_), "cudaMalloc d_div");
    check_cuda(cudaMalloc(&d_dye_, sizeof(float) * n_), "cudaMalloc d_dye");
    check_cuda(cudaMalloc(&d_dye_next_, sizeof(float) * n_), "cudaMalloc d_dye_next");
}

GpuFluid::~GpuFluid() {
    cudaFree(d_vx_);
    cudaFree(d_vy_);
    cudaFree(d_vx_next_);
    cudaFree(d_vy_next_);
    cudaFree(d_p_);
    cudaFree(d_p_next_);
    cudaFree(d_div_);
    cudaFree(d_dye_);
    cudaFree(d_dye_next_);
}

void GpuFluid::load_from_cpu(const CpuFluid& cpu) {
    check_cuda(cudaMemcpy(d_vx_, cpu.vx.data(), sizeof(float) * n_, cudaMemcpyHostToDevice), "copy vx H2D");
    check_cuda(cudaMemcpy(d_vy_, cpu.vy.data(), sizeof(float) * n_, cudaMemcpyHostToDevice), "copy vy H2D");
    check_cuda(cudaMemcpy(d_p_, cpu.pressure.data(), sizeof(float) * n_, cudaMemcpyHostToDevice), "copy p H2D");
    check_cuda(cudaMemcpy(d_div_, cpu.divergence.data(), sizeof(float) * n_, cudaMemcpyHostToDevice), "copy div H2D");
    check_cuda(cudaMemcpy(d_dye_, cpu.dye.data(), sizeof(float) * n_, cudaMemcpyHostToDevice), "copy dye H2D");
}

void GpuFluid::store_to_cpu(CpuFluid& cpu) const {
    check_cuda(cudaMemcpy(cpu.vx.data(), d_vx_, sizeof(float) * n_, cudaMemcpyDeviceToHost), "copy vx D2H");
    check_cuda(cudaMemcpy(cpu.vy.data(), d_vy_, sizeof(float) * n_, cudaMemcpyDeviceToHost), "copy vy D2H");
    check_cuda(cudaMemcpy(cpu.pressure.data(), d_p_, sizeof(float) * n_, cudaMemcpyDeviceToHost), "copy p D2H");
    check_cuda(cudaMemcpy(cpu.divergence.data(), d_div_, sizeof(float) * n_, cudaMemcpyDeviceToHost), "copy div D2H");
    check_cuda(cudaMemcpy(cpu.dye.data(), d_dye_, sizeof(float) * n_, cudaMemcpyDeviceToHost), "copy dye D2H");
}

void GpuFluid::step(const SimConfig& cfg) {
    dim3 block(16, 16);
    dim3 grid((w_ + block.x - 1) / block.x, (h_ + block.y - 1) / block.y);

    advect_velocity_kernel<<<grid, block>>>(d_vx_, d_vy_, d_vx_next_, d_vy_next_, w_, h_, cfg.dt, cfg.dx);
    std::swap(d_vx_, d_vx_next_);
    std::swap(d_vy_, d_vy_next_);
    apply_boundary_velocity_gpu(d_vx_, d_vy_, w_, h_);

    if (diffusion_enabled(cfg)) {
        const float alpha = diffusion_jacobi_alpha(cfg);
        const float inv_beta = 1.0f / (1.0f + 4.0f * alpha);
        diffuse_velocity_gpu(d_vx_, d_vy_, d_vx_next_, d_vy_next_, w_, h_, alpha, inv_beta, cfg.diffusion_iters);
    }

    add_force_kernel<<<grid, block>>>(d_vx_, w_, h_, cfg.dt, cfg.force_amplitude, cfg.force_sigma);
    apply_boundary_velocity_gpu(d_vx_, d_vy_, w_, h_);

    advect_dye_kernel<<<grid, block>>>(d_dye_, d_vx_, d_vy_, d_dye_next_, w_, h_, cfg.dt, cfg.dx);
    std::swap(d_dye_, d_dye_next_);
    add_dye_source_kernel<<<grid, block>>>(d_dye_, w_, h_, cfg.dt, 0.06f);

    divergence_kernel<<<grid, block>>>(d_vx_, d_vy_, d_div_, w_, h_, cfg.dx);

    const int t = 256;
    set_zero_kernel<<<(n_ + t - 1) / t, t>>>(d_p_, n_);
    apply_boundary_pressure_gpu(d_p_, w_, h_);

    const float alpha = cfg.dx * cfg.dx;
    for (int i = 0; i < cfg.pressure_iters; ++i) {
        jacobi_pressure_kernel<<<grid, block>>>(d_p_, d_div_, d_p_next_, w_, h_, alpha);
        std::swap(d_p_, d_p_next_);
        apply_boundary_pressure_gpu(d_p_, w_, h_);
    }

    subtract_gradient_kernel<<<grid, block>>>(d_vx_, d_vy_, d_p_, w_, h_, cfg.dx);
    apply_boundary_velocity_gpu(d_vx_, d_vy_, w_, h_);
}

void GpuFluid::add_mouse_interaction(float norm_x, float norm_y, float force_x, float force_y, float radius_norm, float force_scale, float dye_scale) {
    const float cx = clampf(norm_x, 0.0f, 1.0f) * static_cast<float>(w_ - 1);
    const float cy = clampf(norm_y, 0.0f, 1.0f) * static_cast<float>(h_ - 1);
    const float radius = std::max(1.0f, radius_norm * static_cast<float>(std::min(w_, h_)));

    dim3 block(16, 16);
    dim3 grid((w_ + block.x - 1) / block.x, (h_ + block.y - 1) / block.y);
    mouse_interaction_kernel<<<grid, block>>>(d_vx_, d_vy_, d_dye_, w_, h_, cx, cy, force_x, force_y, radius, force_scale, dye_scale);
    check_cuda(cudaGetLastError(), "mouse_interaction_kernel launch");
    apply_boundary_velocity_gpu(d_vx_, d_vy_, w_, h_);
}

float GpuFluid::time_steps_kernel_ms(const SimConfig& cfg, int warmup, int runs) {
    for (int i = 0; i < warmup; ++i) {
        for (int s = 0; s < cfg.steps; ++s) {
            step(cfg);
        }
    }
    check_cuda(cudaDeviceSynchronize(), "sync after warmup");

    cudaEvent_t start, stop;
    check_cuda(cudaEventCreate(&start), "event create start");
    check_cuda(cudaEventCreate(&stop), "event create stop");

    check_cuda(cudaEventRecord(start), "event record start");
    for (int i = 0; i < runs; ++i) {
        for (int s = 0; s < cfg.steps; ++s) {
            step(cfg);
        }
    }
    check_cuda(cudaEventRecord(stop), "event record stop");
    check_cuda(cudaEventSynchronize(stop), "event sync stop");

    float ms = 0.0f;
    check_cuda(cudaEventElapsedTime(&ms, start, stop), "elapsed time");
    cudaEventDestroy(start);
    cudaEventDestroy(stop);
    return ms / static_cast<float>(runs * cfg.steps);
}

float GpuFluid::time_steps_end_to_end_ms(const CpuFluid& initial, CpuFluid& out, const SimConfig& cfg, int warmup, int runs) {
    for (int i = 0; i < warmup; ++i) {
        load_from_cpu(initial);
        for (int s = 0; s < cfg.steps; ++s) {
            step(cfg);
        }
        store_to_cpu(out);
    }
    check_cuda(cudaDeviceSynchronize(), "sync after warmup e2e");

    cudaEvent_t start, stop;
    check_cuda(cudaEventCreate(&start), "event create start e2e");
    check_cuda(cudaEventCreate(&stop), "event create stop e2e");

    check_cuda(cudaEventRecord(start), "event record start e2e");
    for (int i = 0; i < runs; ++i) {
        load_from_cpu(initial);
        for (int s = 0; s < cfg.steps; ++s) {
            step(cfg);
        }
        store_to_cpu(out);
    }
    check_cuda(cudaEventRecord(stop), "event record stop e2e");
    check_cuda(cudaEventSynchronize(stop), "event sync stop e2e");

    float ms = 0.0f;
    check_cuda(cudaEventElapsedTime(&ms, start, stop), "elapsed time e2e");
    cudaEventDestroy(start);
    cudaEventDestroy(stop);
    return ms / static_cast<float>(runs * cfg.steps);
}

