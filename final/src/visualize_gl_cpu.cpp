#include "visualize_gl.h"

#include "visualize_colormap.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {

float max_velocity_mag(const std::vector<float>& vx, const std::vector<float>& vy) {
    float m = 0.0f;
    for (size_t i = 0; i < vx.size(); ++i) {
        m = std::max(m, std::sqrt(vx[i] * vx[i] + vy[i] * vy[i]));
    }
    return m;
}

float max_abs_scalar(const std::vector<float>& src) {
    float m = 0.0f;
    for (float v : src) {
        m = std::max(m, std::fabs(v));
    }
    return m;
}

}  // namespace

void VisualizeGL::draw_textured_quad() {
    glClear(GL_COLOR_BUFFER_BIT);
    glBindTexture(GL_TEXTURE_2D, tex_);
    glEnable(GL_TEXTURE_2D);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(-1.0f, -1.0f);
    glTexCoord2f(1.0f, 0.0f);
    glVertex2f(1.0f, -1.0f);
    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(1.0f, 1.0f);
    glTexCoord2f(0.0f, 1.0f);
    glVertex2f(-1.0f, 1.0f);
    glEnd();
    glDisable(GL_TEXTURE_2D);
    glfwSwapBuffers(window_);
    ++frame_counter_;
}

void VisualizeGL::set_title(const char* title) {
    if (window_) {
        glfwSetWindowTitle(window_, title);
    }
}

void VisualizeGL::render(const CpuFluid& cpu, RenderField field) {
    if (cpu.w != w_ || cpu.h != h_) {
        throw std::runtime_error("CPU field size does not match visualization size");
    }

    const int n = w_ * h_;
    if (static_cast<int>(host_rgba_.size()) != n * 4) {
        host_rgba_.resize(static_cast<size_t>(n) * 4);
    }

    if ((frame_counter_ % 10) == 0) {
        const float vel_max = max_velocity_mag(cpu.vx, cpu.vy);
        const float p_max = max_abs_scalar(cpu.pressure);
        const float div_max = max_abs_scalar(cpu.divergence);

        const float vel_target = (vel_max > 1e-8f) ? (0.95f / vel_max) : velocity_scale_;
        const float p_target = (p_max > 1e-8f) ? (0.95f / p_max) : pressure_scale_;
        const float div_target = (div_max > 1e-8f) ? (0.95f / div_max) : divergence_scale_;

        const float ema = 0.2f;
        velocity_scale_ = (1.0f - ema) * velocity_scale_ + ema * vel_target;
        pressure_scale_ = (1.0f - ema) * pressure_scale_ + ema * p_target;
        divergence_scale_ = (1.0f - ema) * divergence_scale_ + ema * div_target;
    }

    for (int y = 0; y < h_; ++y) {
        for (int x = 0; x < w_; ++x) {
            const int id = y * w_ + x;
            const size_t o = static_cast<size_t>(id) * 4;
            Rgba8 rgba{};
            if (field == RenderField::Velocity) {
                const float mag = std::sqrt(cpu.vx[id] * cpu.vx[id] + cpu.vy[id] * cpu.vy[id]);
                rgba = to_rgba_heat_host(mag * velocity_scale_);
            } else if (field == RenderField::Pressure) {
                const float t = 0.5f + 0.5f * cpu.pressure[id] * pressure_scale_;
                rgba = to_rgba_diverging_host(t);
            } else if (field == RenderField::Divergence) {
                const float t = 0.5f + 0.5f * cpu.divergence[id] * divergence_scale_;
                rgba = to_rgba_diverging_host(t);
            } else {
                rgba = to_rgba_dye_intense_host(cpu.dye[id]);
            }
            host_rgba_[o + 0] = rgba.r;
            host_rgba_[o + 1] = rgba.g;
            host_rgba_[o + 2] = rgba.b;
            host_rgba_[o + 3] = rgba.a;
        }
    }

    glBindTexture(GL_TEXTURE_2D, tex_);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w_, h_, GL_RGBA, GL_UNSIGNED_BYTE, host_rgba_.data());
    draw_textured_quad();
}
