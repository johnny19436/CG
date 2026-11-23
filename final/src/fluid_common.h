#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

struct SimConfig {
    int width = 256;
    int height = 256;
    int pressure_iters = 40;
    int diffusion_iters = 20;
    int steps = 100;
    float dt = 0.1f;
    float dx = 1.0f;
    float viscosity = 0.0f;
    float force_amplitude = 4.0f;
    float force_sigma = 0.08f;
};

// Stam-style implicit diffusion: a = dt * viscosity * N^2, N = min(width, height).
inline float diffusion_jacobi_alpha(const SimConfig& cfg) {
    const int n = std::min(cfg.width, cfg.height);
    return cfg.dt * cfg.viscosity * static_cast<float>(n) * static_cast<float>(n);
}

inline bool diffusion_enabled(const SimConfig& cfg) {
    return cfg.viscosity > 0.0f && cfg.diffusion_iters > 0;
}

struct ErrorMetrics {
    float max_abs = 0.0f;
    float mean_abs = 0.0f;
    float relative_l2 = 0.0f;
};

#ifdef __CUDACC__
#define HD __host__ __device__
#else
#define HD
#endif

HD inline int idx(int x, int y, int w) {
    return y * w + x;
}

inline float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(v, hi));
}

#undef HD

inline void compute_error_metrics(const std::vector<float>& a, const std::vector<float>& b, ErrorMetrics& out) {
    const std::size_t n = a.size();
    double sum_abs = 0.0;
    double sum_sq_num = 0.0;
    double sum_sq_den = 0.0;
    float max_abs = 0.0f;

    for (std::size_t i = 0; i < n; ++i) {
        const float d = std::fabs(a[i] - b[i]);
        max_abs = std::max(max_abs, d);
        sum_abs += static_cast<double>(d);
        sum_sq_num += static_cast<double>(d) * static_cast<double>(d);
        sum_sq_den += static_cast<double>(a[i]) * static_cast<double>(a[i]);
    }

    out.max_abs = max_abs;
    out.mean_abs = static_cast<float>(sum_abs / static_cast<double>(n));
    if (sum_sq_den <= 1e-20) {
        out.relative_l2 = static_cast<float>(std::sqrt(sum_sq_num));
    } else {
        out.relative_l2 = static_cast<float>(std::sqrt(sum_sq_num / sum_sq_den));
    }
}

