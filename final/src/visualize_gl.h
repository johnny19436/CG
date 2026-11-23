#pragma once

#include "cpu_sim.h"
#include "gpu_sim.h"

#include <vector>

struct GLFWwindow;
struct cudaGraphicsResource;

enum class RenderField {
    Velocity = 0,
    Pressure = 1,
    Divergence = 2,
    Dye = 3
};

class VisualizeGL {
public:
    // use_cuda_interop: true for GPU sim (CUDA PBO path); false for CPU sim (host RGBA upload).
    VisualizeGL(int width, int height, bool vsync, bool use_cuda_interop);
    ~VisualizeGL();

    VisualizeGL(const VisualizeGL&) = delete;
    VisualizeGL& operator=(const VisualizeGL&) = delete;

    bool should_close() const;
    void poll_events();
    void set_title(const char* title);
    void render(const GpuFluid& gpu, RenderField field);
    void render(const CpuFluid& cpu, RenderField field);
    GLFWwindow* window() const { return window_; }

private:
    void draw_textured_quad();

    int w_;
    int h_;
    int frame_counter_;
    bool use_cuda_interop_;
    GLFWwindow* window_;
    unsigned int tex_;
    unsigned int pbo_;
    cudaGraphicsResource* cuda_pbo_resource_;
    std::vector<unsigned char> host_rgba_;
    float velocity_scale_;
    float pressure_scale_;
    float divergence_scale_;
};
