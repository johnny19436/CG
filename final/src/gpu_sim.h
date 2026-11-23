#pragma once

#include "fluid_common.h"

struct CpuFluid;

class GpuFluid {
public:
    GpuFluid(int width, int height);
    ~GpuFluid();

    GpuFluid(const GpuFluid&) = delete;
    GpuFluid& operator=(const GpuFluid&) = delete;

    void load_from_cpu(const CpuFluid& cpu);
    void store_to_cpu(CpuFluid& cpu) const;
    void step(const SimConfig& cfg);
    void add_mouse_interaction(float norm_x, float norm_y, float force_x, float force_y, float radius_norm, float force_scale, float dye_scale);
    float time_steps_kernel_ms(const SimConfig& cfg, int warmup, int runs);
    float time_steps_end_to_end_ms(const CpuFluid& initial, CpuFluid& out, const SimConfig& cfg, int warmup, int runs);
    int width() const { return w_; }
    int height() const { return h_; }
    const float* device_vx() const { return d_vx_; }
    const float* device_vy() const { return d_vy_; }
    const float* device_pressure() const { return d_p_; }
    const float* device_divergence() const { return d_div_; }
    const float* device_dye() const { return d_dye_; }

private:
    int w_;
    int h_;
    int n_;

    float* d_vx_;
    float* d_vy_;
    float* d_vx_next_;
    float* d_vy_next_;
    float* d_p_;
    float* d_p_next_;
    float* d_div_;
    float* d_dye_;
    float* d_dye_next_;
};

