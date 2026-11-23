#pragma once

#include "fluid_common.h"

struct CpuFluid {
    int w;
    int h;
    std::vector<float> vx;
    std::vector<float> vy;
    std::vector<float> vx_next;
    std::vector<float> vy_next;
    std::vector<float> pressure;
    std::vector<float> pressure_next;
    std::vector<float> divergence;
    std::vector<float> dye;
    std::vector<float> dye_next;

    explicit CpuFluid(int width, int height);
    void initialize_deterministic();
    void step(const SimConfig& cfg);
    void add_mouse_interaction(float norm_x, float norm_y, float force_x, float force_y, float radius_norm, float force_scale,
                               float dye_scale);
};

