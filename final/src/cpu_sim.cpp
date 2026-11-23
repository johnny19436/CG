#include "cpu_sim.h"

#include <cmath>
#include <utility>

namespace {

float sample_bilinear(const std::vector<float>& field, int w, int h, float x, float y) {
    x = clampf(x, 1.0f, static_cast<float>(w - 2));
    y = clampf(y, 1.0f, static_cast<float>(h - 2));

    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = std::min(x0 + 1, w - 2);
    const int y1 = std::min(y0 + 1, h - 2);
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

void apply_boundary_vx(std::vector<float>& vx, int w, int h) {
    for (int y = 1; y < h - 1; ++y) {
        vx[idx(0, y, w)] = -vx[idx(1, y, w)];
        vx[idx(w - 1, y, w)] = -vx[idx(w - 2, y, w)];
    }
    for (int x = 1; x < w - 1; ++x) {
        vx[idx(x, 0, w)] = vx[idx(x, 1, w)];
        vx[idx(x, h - 1, w)] = vx[idx(x, h - 2, w)];
    }
    vx[idx(0, 0, w)] = 0.5f * (vx[idx(1, 0, w)] + vx[idx(0, 1, w)]);
    vx[idx(w - 1, 0, w)] = 0.5f * (vx[idx(w - 2, 0, w)] + vx[idx(w - 1, 1, w)]);
    vx[idx(0, h - 1, w)] = 0.5f * (vx[idx(1, h - 1, w)] + vx[idx(0, h - 2, w)]);
    vx[idx(w - 1, h - 1, w)] = 0.5f * (vx[idx(w - 2, h - 1, w)] + vx[idx(w - 1, h - 2, w)]);
}

void apply_boundary_vy(std::vector<float>& vy, int w, int h) {
    for (int y = 1; y < h - 1; ++y) {
        vy[idx(0, y, w)] = vy[idx(1, y, w)];
        vy[idx(w - 1, y, w)] = vy[idx(w - 2, y, w)];
    }
    for (int x = 1; x < w - 1; ++x) {
        vy[idx(x, 0, w)] = -vy[idx(x, 1, w)];
        vy[idx(x, h - 1, w)] = -vy[idx(x, h - 2, w)];
    }
    vy[idx(0, 0, w)] = 0.5f * (vy[idx(1, 0, w)] + vy[idx(0, 1, w)]);
    vy[idx(w - 1, 0, w)] = 0.5f * (vy[idx(w - 2, 0, w)] + vy[idx(w - 1, 1, w)]);
    vy[idx(0, h - 1, w)] = 0.5f * (vy[idx(1, h - 1, w)] + vy[idx(0, h - 2, w)]);
    vy[idx(w - 1, h - 1, w)] = 0.5f * (vy[idx(w - 2, h - 1, w)] + vy[idx(w - 1, h - 2, w)]);
}

void apply_boundary_velocity(std::vector<float>& vx, std::vector<float>& vy, int w, int h) {
    apply_boundary_vx(vx, w, h);
    apply_boundary_vy(vy, w, h);
}

void diffuse_velocity_component(std::vector<float>& x, std::vector<float>& x0, std::vector<float>& scratch, int w, int h, float alpha,
                                float inv_beta, int iters, bool component_is_vx) {
    for (int y = 1; y < h - 1; ++y) {
        for (int x_i = 1; x_i < w - 1; ++x_i) {
            x0[idx(x_i, y, w)] = x[idx(x_i, y, w)];
        }
    }

    for (int k = 0; k < iters; ++k) {
        for (int y = 1; y < h - 1; ++y) {
            for (int x_i = 1; x_i < w - 1; ++x_i) {
                const int id = idx(x_i, y, w);
                const float sum_n = x[idx(x_i + 1, y, w)] + x[idx(x_i - 1, y, w)] + x[idx(x_i, y + 1, w)] +
                                    x[idx(x_i, y - 1, w)];
                scratch[id] = (x0[id] + alpha * sum_n) * inv_beta;
            }
        }
        for (int y = 1; y < h - 1; ++y) {
            for (int x_i = 1; x_i < w - 1; ++x_i) {
                x[idx(x_i, y, w)] = scratch[idx(x_i, y, w)];
            }
        }
        if (component_is_vx) {
            apply_boundary_vx(x, w, h);
        } else {
            apply_boundary_vy(x, w, h);
        }
    }
}

void diffuse_velocity(std::vector<float>& vx, std::vector<float>& vy, std::vector<float>& vx_next, std::vector<float>& vy_next, int w, int h,
                      float alpha, float inv_beta, int iters) {
    diffuse_velocity_component(vx, vx_next, vy_next, w, h, alpha, inv_beta, iters, true);
    diffuse_velocity_component(vy, vy_next, vx_next, w, h, alpha, inv_beta, iters, false);
}

void apply_boundary_pressure(std::vector<float>& p, int w, int h) {
    for (int y = 1; y < h - 1; ++y) {
        p[idx(0, y, w)] = p[idx(1, y, w)];
        p[idx(w - 1, y, w)] = p[idx(w - 2, y, w)];
    }
    for (int x = 1; x < w - 1; ++x) {
        p[idx(x, 0, w)] = p[idx(x, 1, w)];
        p[idx(x, h - 1, w)] = p[idx(x, h - 2, w)];
    }

    p[idx(0, 0, w)] = 0.5f * (p[idx(1, 0, w)] + p[idx(0, 1, w)]);
    p[idx(w - 1, 0, w)] = 0.5f * (p[idx(w - 2, 0, w)] + p[idx(w - 1, 1, w)]);
    p[idx(0, h - 1, w)] = 0.5f * (p[idx(1, h - 1, w)] + p[idx(0, h - 2, w)]);
    p[idx(w - 1, h - 1, w)] = 0.5f * (p[idx(w - 2, h - 1, w)] + p[idx(w - 1, h - 2, w)]);
}

}  // namespace

CpuFluid::CpuFluid(int width, int height)
    : w(width),
      h(height),
      vx(static_cast<std::size_t>(w) * h, 0.0f),
      vy(static_cast<std::size_t>(w) * h, 0.0f),
      vx_next(static_cast<std::size_t>(w) * h, 0.0f),
      vy_next(static_cast<std::size_t>(w) * h, 0.0f),
      pressure(static_cast<std::size_t>(w) * h, 0.0f),
      pressure_next(static_cast<std::size_t>(w) * h, 0.0f),
      divergence(static_cast<std::size_t>(w) * h, 0.0f),
      dye(static_cast<std::size_t>(w) * h, 0.0f),
      dye_next(static_cast<std::size_t>(w) * h, 0.0f) {}

void CpuFluid::initialize_deterministic() {
    const float pi = 3.14159265359f;
    const float init_vel_scale = 0.35f;
    const float cx = 0.5f * static_cast<float>(w - 1);
    const float cy = 0.5f * static_cast<float>(h - 1);
    const float sigma_cells = 0.06f * static_cast<float>(std::min(w, h));
    const float inv_two_sigma2 = 1.0f / (2.0f * sigma_cells * sigma_cells + 1e-8f);
    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            const float fx = static_cast<float>(x - 1) / static_cast<float>(w - 3);
            const float fy = static_cast<float>(y - 1) / static_cast<float>(h - 3);
            vx[idx(x, y, w)] = init_vel_scale * std::sin(2.0f * pi * fy);
            vy[idx(x, y, w)] = init_vel_scale * std::cos(2.0f * pi * fx);
            const float dx_c = static_cast<float>(x) - cx;
            const float dy_c = static_cast<float>(y) - cy;
            dye[idx(x, y, w)] = std::exp(-(dx_c * dx_c + dy_c * dy_c) * inv_two_sigma2);
        }
    }
    apply_boundary_velocity(vx, vy, w, h);
}

void CpuFluid::step(const SimConfig& cfg) {
    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            const int id = idx(x, y, w);
            const float back_x = static_cast<float>(x) - cfg.dt * vx[id] / cfg.dx;
            const float back_y = static_cast<float>(y) - cfg.dt * vy[id] / cfg.dx;
            vx_next[id] = sample_bilinear(vx, w, h, back_x, back_y);
            vy_next[id] = sample_bilinear(vy, w, h, back_x, back_y);
        }
    }
    std::swap(vx, vx_next);
    std::swap(vy, vy_next);
    apply_boundary_velocity(vx, vy, w, h);

    if (diffusion_enabled(cfg)) {
        const float alpha = diffusion_jacobi_alpha(cfg);
        const float inv_beta = 1.0f / (1.0f + 4.0f * alpha);
        diffuse_velocity(vx, vy, vx_next, vy_next, w, h, alpha, inv_beta, cfg.diffusion_iters);
        apply_boundary_velocity(vx, vy, w, h);
    }

    const float cx = 0.5f * static_cast<float>(w - 1);
    const float cy = 0.5f * static_cast<float>(h - 1);
    const float sigma_cells = cfg.force_sigma * static_cast<float>(std::min(w, h));
    const float inv_two_sigma2 = 1.0f / (2.0f * sigma_cells * sigma_cells + 1e-8f);

    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            const float dx_c = static_cast<float>(x) - cx;
            const float dy_c = static_cast<float>(y) - cy;
            const float g = std::exp(-(dx_c * dx_c + dy_c * dy_c) * inv_two_sigma2);
            vx[idx(x, y, w)] += cfg.dt * cfg.force_amplitude * g;
        }
    }
    apply_boundary_velocity(vx, vy, w, h);

    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            const int id = idx(x, y, w);
            const float back_x = static_cast<float>(x) - cfg.dt * vx[id] / cfg.dx;
            const float back_y = static_cast<float>(y) - cfg.dt * vy[id] / cfg.dx;
            const float transported = sample_bilinear(dye, w, h, back_x, back_y);
            const float dx_c = static_cast<float>(x) - cx;
            const float dy_c = static_cast<float>(y) - cy;
            const float src = std::exp(-(dx_c * dx_c + dy_c * dy_c) * inv_two_sigma2);
            dye_next[id] = clampf(0.995f * transported + 0.15f * cfg.dt * src, 0.0f, 1.0f);
        }
    }
    std::swap(dye, dye_next);

    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            divergence[idx(x, y, w)] = 0.5f / cfg.dx *
                                       (vx[idx(x + 1, y, w)] - vx[idx(x - 1, y, w)] +
                                        vy[idx(x, y + 1, w)] - vy[idx(x, y - 1, w)]);
        }
    }

    std::fill(pressure.begin(), pressure.end(), 0.0f);
    apply_boundary_pressure(pressure, w, h);

    const float alpha = cfg.dx * cfg.dx;
    for (int it = 0; it < cfg.pressure_iters; ++it) {
        for (int y = 1; y < h - 1; ++y) {
            for (int x = 1; x < w - 1; ++x) {
                pressure_next[idx(x, y, w)] =
                    (pressure[idx(x + 1, y, w)] + pressure[idx(x - 1, y, w)] +
                     pressure[idx(x, y + 1, w)] + pressure[idx(x, y - 1, w)] -
                     alpha * divergence[idx(x, y, w)]) *
                    0.25f;
            }
        }
        std::swap(pressure, pressure_next);
        apply_boundary_pressure(pressure, w, h);
    }

    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            vx[idx(x, y, w)] -= 0.5f / cfg.dx * (pressure[idx(x + 1, y, w)] - pressure[idx(x - 1, y, w)]);
            vy[idx(x, y, w)] -= 0.5f / cfg.dx * (pressure[idx(x, y + 1, w)] - pressure[idx(x, y - 1, w)]);
        }
    }
    apply_boundary_velocity(vx, vy, w, h);
}

void CpuFluid::add_mouse_interaction(float norm_x, float norm_y, float force_x, float force_y, float radius_norm, float force_scale,
                                     float dye_scale) {
    const float cx = clampf(norm_x, 0.0f, 1.0f) * static_cast<float>(w - 1);
    const float cy = clampf(norm_y, 0.0f, 1.0f) * static_cast<float>(h - 1);
    const float radius = std::max(1.0f, radius_norm * static_cast<float>(std::min(w, h)));
    const float inv_two_sigma2 = 1.0f / (2.0f * radius * radius + 1e-8f);

    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            const float dx_c = static_cast<float>(x) - cx;
            const float dy_c = static_cast<float>(y) - cy;
            const float wgt = std::exp(-(dx_c * dx_c + dy_c * dy_c) * inv_two_sigma2);
            const int id = idx(x, y, w);
            vx[id] += force_scale * force_x * wgt;
            vy[id] += force_scale * force_y * wgt;
            dye[id] = clampf(dye[id] + dye_scale * wgt, 0.0f, 1.0f);
        }
    }
    apply_boundary_velocity(vx, vy, w, h);
}

