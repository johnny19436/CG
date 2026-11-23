#include "visualize_gl.h"

#include <cuda_gl_interop.h>
#include <cuda_runtime.h>

#include <GLFW/glfw3.h>
#include <GL/glext.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace {

PFNGLGENBUFFERSPROC p_glGenBuffers = nullptr;
PFNGLBINDBUFFERPROC p_glBindBuffer = nullptr;
PFNGLBUFFERDATAPROC p_glBufferData = nullptr;
PFNGLDELETEBUFFERSPROC p_glDeleteBuffers = nullptr;

void load_gl_buffer_functions() {
    p_glGenBuffers = reinterpret_cast<PFNGLGENBUFFERSPROC>(glfwGetProcAddress("glGenBuffers"));
    p_glBindBuffer = reinterpret_cast<PFNGLBINDBUFFERPROC>(glfwGetProcAddress("glBindBuffer"));
    p_glBufferData = reinterpret_cast<PFNGLBUFFERDATAPROC>(glfwGetProcAddress("glBufferData"));
    p_glDeleteBuffers = reinterpret_cast<PFNGLDELETEBUFFERSPROC>(glfwGetProcAddress("glDeleteBuffers"));
    if (!p_glGenBuffers || !p_glBindBuffer || !p_glBufferData || !p_glDeleteBuffers) {
        throw std::runtime_error("Failed to load OpenGL buffer functions");
    }
}

void check_cuda(cudaError_t err, const char* msg) {
    if (err != cudaSuccess) {
        throw std::runtime_error(std::string(msg) + ": " + cudaGetErrorString(err));
    }
}

__device__ float clamp01(float v) {
    return fminf(1.0f, fmaxf(0.0f, v));
}

__device__ uchar4 to_rgba_heat(float t) {
    t = clamp01(t);
    const float r = clamp01(1.5f * t - 0.5f);
    const float g = clamp01(1.5f - fabsf(2.0f * t - 1.0f));
    const float b = clamp01(1.0f - 1.5f * t);
    return make_uchar4(static_cast<unsigned char>(255.0f * r), static_cast<unsigned char>(255.0f * g),
                       static_cast<unsigned char>(255.0f * b), 255);
}

__device__ uchar4 to_rgba_diverging(float t) {
    t = clamp01(t);
    const float r = clamp01(2.0f * t - 0.1f);
    const float b = clamp01(2.0f * (1.0f - t) - 0.1f);
    const float g = clamp01(1.25f - 2.5f * fabsf(t - 0.5f));
    return make_uchar4(static_cast<unsigned char>(255.0f * r), static_cast<unsigned char>(255.0f * g),
                       static_cast<unsigned char>(255.0f * b), 255);
}

__device__ uchar4 to_rgba_dye_intense(float d) {
    // Inferno/magma-like palette: deep blue/purple -> magenta/red -> orange/yellow.
    d = clamp01(d);
    const float t = powf(d, 0.58f);
    const float t2 = t * t;
    const float t3 = t2 * t;

    // Polynomial fit tuned for vivid smoke-like contrast.
    const float r = clamp01(0.05f + 2.35f * t - 1.55f * t2 + 0.35f * t3);
    const float g = clamp01(0.00f + 0.28f * t + 1.85f * t2 - 1.10f * t3);
    const float b = clamp01(0.16f + 1.35f * t - 2.55f * t2 + 1.05f * t3);
    return make_uchar4(static_cast<unsigned char>(255.0f * r), static_cast<unsigned char>(255.0f * g),
                       static_cast<unsigned char>(255.0f * b), 255);
}

__global__ void render_velocity_kernel(uchar4* out, const float* vx, const float* vy, int w, int h, float scale) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= w || y >= h) {
        return;
    }
    const int id = y * w + x;
    const float mag = sqrtf(vx[id] * vx[id] + vy[id] * vy[id]);
    out[id] = to_rgba_heat(mag * scale);
}

__global__ void render_scalar_centered_kernel(uchar4* out, const float* src, int w, int h, float scale) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= w || y >= h) {
        return;
    }
    const int id = y * w + x;
    const float t = 0.5f + 0.5f * src[id] * scale;
    out[id] = to_rgba_diverging(t);
}

__global__ void render_dye_kernel(uchar4* out, const float* dye, int w, int h) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= w || y >= h) {
        return;
    }
    const int id = y * w + x;
    out[id] = to_rgba_dye_intense(dye[id]);
}

__global__ void max_velocity_mag_kernel(const float* vx, const float* vy, int n, float* out_max) {
    __shared__ float shared[256];
    const int tid = threadIdx.x;
    const int i = blockIdx.x * blockDim.x + tid;
    float v = 0.0f;
    if (i < n) {
        v = sqrtf(vx[i] * vx[i] + vy[i] * vy[i]);
    }
    shared[tid] = v;
    __syncthreads();
    for (int stride = blockDim.x / 2; stride > 0; stride >>= 1) {
        if (tid < stride) {
            shared[tid] = fmaxf(shared[tid], shared[tid + stride]);
        }
        __syncthreads();
    }
    if (tid == 0) {
        atomicMax(reinterpret_cast<int*>(out_max), __float_as_int(shared[0]));
    }
}

__global__ void max_abs_kernel(const float* src, int n, float* out_max) {
    __shared__ float shared[256];
    const int tid = threadIdx.x;
    const int i = blockIdx.x * blockDim.x + tid;
    float v = 0.0f;
    if (i < n) {
        v = fabsf(src[i]);
    }
    shared[tid] = v;
    __syncthreads();
    for (int stride = blockDim.x / 2; stride > 0; stride >>= 1) {
        if (tid < stride) {
            shared[tid] = fmaxf(shared[tid], shared[tid + stride]);
        }
        __syncthreads();
    }
    if (tid == 0) {
        atomicMax(reinterpret_cast<int*>(out_max), __float_as_int(shared[0]));
    }
}

float compute_velocity_max_abs(const GpuFluid& gpu, int n) {
    float* d_max = nullptr;
    check_cuda(cudaMalloc(&d_max, sizeof(float)), "cudaMalloc d_max velocity");
    check_cuda(cudaMemset(d_max, 0, sizeof(float)), "cudaMemset d_max velocity");
    const int t = 256;
    const int b = (n + t - 1) / t;
    max_velocity_mag_kernel<<<b, t>>>(gpu.device_vx(), gpu.device_vy(), n, d_max);
    check_cuda(cudaGetLastError(), "max_velocity_mag_kernel launch");
    float h_max = 0.0f;
    check_cuda(cudaMemcpy(&h_max, d_max, sizeof(float), cudaMemcpyDeviceToHost), "copy velocity max");
    cudaFree(d_max);
    return h_max;
}

float compute_scalar_max_abs(const float* src, int n) {
    float* d_max = nullptr;
    check_cuda(cudaMalloc(&d_max, sizeof(float)), "cudaMalloc d_max scalar");
    check_cuda(cudaMemset(d_max, 0, sizeof(float)), "cudaMemset d_max scalar");
    const int t = 256;
    const int b = (n + t - 1) / t;
    max_abs_kernel<<<b, t>>>(src, n, d_max);
    check_cuda(cudaGetLastError(), "max_abs_kernel launch");
    float h_max = 0.0f;
    check_cuda(cudaMemcpy(&h_max, d_max, sizeof(float), cudaMemcpyDeviceToHost), "copy scalar max");
    cudaFree(d_max);
    return h_max;
}

}  // namespace

VisualizeGL::VisualizeGL(int width, int height, bool vsync, bool use_cuda_interop)
    : w_(width),
      h_(height),
      frame_counter_(0),
      use_cuda_interop_(use_cuda_interop),
      window_(nullptr),
      tex_(0),
      pbo_(0),
      cuda_pbo_resource_(nullptr),
      velocity_scale_(20.0f),
      pressure_scale_(10.0f),
      divergence_scale_(40.0f) {
    if (!glfwInit()) {
        throw std::runtime_error("glfwInit failed");
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    window_ = glfwCreateWindow(w_, h_, "Fluid Visualization", nullptr, nullptr);
    if (!window_) {
        glfwTerminate();
        throw std::runtime_error("glfwCreateWindow failed");
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(vsync ? 1 : 0);
    load_gl_buffer_functions();

    glViewport(0, 0, w_, h_);

    glGenTextures(1, &tex_);
    glBindTexture(GL_TEXTURE_2D, tex_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w_, h_, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);

    if (use_cuda_interop_) {
        p_glGenBuffers(1, &pbo_);
        p_glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo_);
        p_glBufferData(GL_PIXEL_UNPACK_BUFFER, static_cast<GLsizeiptr>(sizeof(uchar4) * w_ * h_), nullptr, GL_DYNAMIC_DRAW);
        p_glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
        check_cuda(cudaGraphicsGLRegisterBuffer(&cuda_pbo_resource_, pbo_, cudaGraphicsRegisterFlagsWriteDiscard),
                   "cudaGraphicsGLRegisterBuffer");
    }
}

VisualizeGL::~VisualizeGL() {
    if (cuda_pbo_resource_) {
        cudaGraphicsUnregisterResource(cuda_pbo_resource_);
        cuda_pbo_resource_ = nullptr;
    }
    if (pbo_ != 0) {
        p_glDeleteBuffers(1, &pbo_);
    }
    if (tex_ != 0) {
        glDeleteTextures(1, &tex_);
    }
    if (window_) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }
    glfwTerminate();
}

bool VisualizeGL::should_close() const {
    return glfwWindowShouldClose(window_);
}

void VisualizeGL::poll_events() {
    glfwPollEvents();
}

void VisualizeGL::render(const GpuFluid& gpu, RenderField field) {
    if (!use_cuda_interop_) {
        throw std::runtime_error("GPU render requires CUDA–OpenGL interop (use --backend gpu)");
    }
    if (gpu.width() != w_ || gpu.height() != h_) {
        throw std::runtime_error("GPU field size does not match visualization size");
    }

    check_cuda(cudaGraphicsMapResources(1, &cuda_pbo_resource_), "cudaGraphicsMapResources");
    uchar4* dev_ptr = nullptr;
    size_t mapped_size = 0;
    check_cuda(cudaGraphicsResourceGetMappedPointer(reinterpret_cast<void**>(&dev_ptr), &mapped_size, cuda_pbo_resource_),
               "cudaGraphicsResourceGetMappedPointer");

    dim3 block(16, 16);
    dim3 grid((w_ + block.x - 1) / block.x, (h_ + block.y - 1) / block.y);
    const int n = w_ * h_;

    // Re-estimate dynamic scales periodically to avoid color saturation.
    if ((frame_counter_ % 10) == 0) {
        const float vel_max = compute_velocity_max_abs(gpu, n);
        const float p_max = compute_scalar_max_abs(gpu.device_pressure(), n);
        const float div_max = compute_scalar_max_abs(gpu.device_divergence(), n);

        const float vel_target = (vel_max > 1e-8f) ? (0.95f / vel_max) : velocity_scale_;
        const float p_target = (p_max > 1e-8f) ? (0.95f / p_max) : pressure_scale_;
        const float div_target = (div_max > 1e-8f) ? (0.95f / div_max) : divergence_scale_;

        const float ema = 0.2f;
        velocity_scale_ = (1.0f - ema) * velocity_scale_ + ema * vel_target;
        pressure_scale_ = (1.0f - ema) * pressure_scale_ + ema * p_target;
        divergence_scale_ = (1.0f - ema) * divergence_scale_ + ema * div_target;
    }

    if (field == RenderField::Velocity) {
        render_velocity_kernel<<<grid, block>>>(dev_ptr, gpu.device_vx(), gpu.device_vy(), w_, h_, velocity_scale_);
    } else if (field == RenderField::Pressure) {
        render_scalar_centered_kernel<<<grid, block>>>(dev_ptr, gpu.device_pressure(), w_, h_, pressure_scale_);
    } else if (field == RenderField::Divergence) {
        render_scalar_centered_kernel<<<grid, block>>>(dev_ptr, gpu.device_divergence(), w_, h_, divergence_scale_);
    } else {
        render_dye_kernel<<<grid, block>>>(dev_ptr, gpu.device_dye(), w_, h_);
    }

    check_cuda(cudaGetLastError(), "render kernel launch");
    check_cuda(cudaGraphicsUnmapResources(1, &cuda_pbo_resource_), "cudaGraphicsUnmapResources");

    glBindTexture(GL_TEXTURE_2D, tex_);
    p_glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo_);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w_, h_, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    p_glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);

    draw_textured_quad();
}

