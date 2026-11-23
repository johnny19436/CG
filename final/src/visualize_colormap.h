#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

struct Rgba8 {
    unsigned char r;
    unsigned char g;
    unsigned char b;
    unsigned char a;
};

inline float clamp01_host(float v) {
    return std::max(0.0f, std::min(1.0f, v));
}

inline Rgba8 to_rgba_heat_host(float t) {
    t = clamp01_host(t);
    return Rgba8{static_cast<unsigned char>(255.0f * clamp01_host(1.5f * t - 0.5f)),
                 static_cast<unsigned char>(255.0f * clamp01_host(1.5f - std::fabs(2.0f * t - 1.0f))),
                 static_cast<unsigned char>(255.0f * clamp01_host(1.0f - 1.5f * t)), 255};
}

inline Rgba8 to_rgba_diverging_host(float t) {
    t = clamp01_host(t);
    return Rgba8{static_cast<unsigned char>(255.0f * clamp01_host(2.0f * t - 0.1f)),
                 static_cast<unsigned char>(255.0f * clamp01_host(1.25f - 2.5f * std::fabs(t - 0.5f))),
                 static_cast<unsigned char>(255.0f * clamp01_host(2.0f * (1.0f - t) - 0.1f)), 255};
}

inline Rgba8 to_rgba_dye_intense_host(float d) {
    d = clamp01_host(d);
    const float t = std::pow(d, 0.58f);
    const float t2 = t * t;
    const float t3 = t2 * t;
    const float r = clamp01_host(0.05f + 2.35f * t - 1.55f * t2 + 0.35f * t3);
    const float g = clamp01_host(0.00f + 0.28f * t + 1.85f * t2 - 1.10f * t3);
    const float b = clamp01_host(0.16f + 1.35f * t - 2.55f * t2 + 1.05f * t3);
    return Rgba8{static_cast<unsigned char>(255.0f * r), static_cast<unsigned char>(255.0f * g),
                 static_cast<unsigned char>(255.0f * b), 255};
}
