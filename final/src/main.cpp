#include "cpu_sim.h"
#include "fluid_common.h"
#include "gpu_sim.h"
#include "visualize_gl.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

int arg_to_int(const char* s) {
    return std::atoi(s);
}

float arg_to_float(const char* s) {
    return std::strtof(s, nullptr);
}

void parse_common_args(int argc, char** argv, SimConfig& cfg, int start_idx) {
    for (int i = start_idx; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--width" && i + 1 < argc) {
            cfg.width = arg_to_int(argv[++i]);
        } else if (a == "--height" && i + 1 < argc) {
            cfg.height = arg_to_int(argv[++i]);
        } else if (a == "--steps" && i + 1 < argc) {
            cfg.steps = arg_to_int(argv[++i]);
        } else if (a == "--iters" && i + 1 < argc) {
            cfg.pressure_iters = arg_to_int(argv[++i]);
        } else if (a == "--diffusion-iters" && i + 1 < argc) {
            cfg.diffusion_iters = arg_to_int(argv[++i]);
        } else if (a == "--viscosity" && i + 1 < argc) {
            cfg.viscosity = arg_to_float(argv[++i]);
        } else if (a == "--dt" && i + 1 < argc) {
            cfg.dt = arg_to_float(argv[++i]);
        } else if (a == "--dx" && i + 1 < argc) {
            cfg.dx = arg_to_float(argv[++i]);
        } else if (a == "--force-amplitude" && i + 1 < argc) {
            cfg.force_amplitude = arg_to_float(argv[++i]);
        } else if (a == "--force-sigma" && i + 1 < argc) {
            cfg.force_sigma = arg_to_float(argv[++i]);
        }
    }
}

void run_verify(int argc, char** argv) {
    SimConfig cfg;
    parse_common_args(argc, argv, cfg, 2);

    CpuFluid cpu(cfg.width, cfg.height);
    cpu.initialize_deterministic();
    CpuFluid gpu_result = cpu;

    GpuFluid gpu(cfg.width, cfg.height);
    gpu.load_from_cpu(cpu);

    for (int i = 0; i < cfg.steps; ++i) {
        cpu.step(cfg);
        gpu.step(cfg);
    }
    gpu.store_to_cpu(gpu_result);

    ErrorMetrics vx_e, vy_e, p_e, div_e;
    compute_error_metrics(cpu.vx, gpu_result.vx, vx_e);
    compute_error_metrics(cpu.vy, gpu_result.vy, vy_e);
    compute_error_metrics(cpu.pressure, gpu_result.pressure, p_e);
    compute_error_metrics(cpu.divergence, gpu_result.divergence, div_e);

    std::cout << "Correctness Check\n";
    std::cout << "Grid: " << cfg.width << "x" << cfg.height << ", Steps: " << cfg.steps << ", Pressure Iters: " << cfg.pressure_iters
              << ", Diffusion: " << (diffusion_enabled(cfg) ? "on" : "off");
    if (diffusion_enabled(cfg)) {
        std::cout << " (viscosity=" << cfg.viscosity << ", diffusion_iters=" << cfg.diffusion_iters << ")";
    }
    std::cout << "\n\n";
    std::cout << std::left << std::setw(12) << "Field" << std::setw(16) << "MaxAbs" << std::setw(16) << "MeanAbs" << "RelativeL2\n";
    std::cout << std::left << std::setw(12) << "vel_x" << std::setw(16) << vx_e.max_abs << std::setw(16) << vx_e.mean_abs << vx_e.relative_l2 << "\n";
    std::cout << std::left << std::setw(12) << "vel_y" << std::setw(16) << vy_e.max_abs << std::setw(16) << vy_e.mean_abs << vy_e.relative_l2 << "\n";
    std::cout << std::left << std::setw(12) << "pressure" << std::setw(16) << p_e.max_abs << std::setw(16) << p_e.mean_abs << p_e.relative_l2 << "\n";
    std::cout << std::left << std::setw(12) << "divergence" << std::setw(16) << div_e.max_abs << std::setw(16) << div_e.mean_abs << div_e.relative_l2 << "\n";
}

void run_bench(int argc, char** argv) {
    SimConfig base;
    int warmup = 5;
    int runs = 50;

    parse_common_args(argc, argv, base, 2);
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--warmup" && i + 1 < argc) {
            warmup = arg_to_int(argv[++i]);
        } else if (a == "--runs" && i + 1 < argc) {
            runs = arg_to_int(argv[++i]);
        }
    }

    std::vector<int> sizes = {128, 256, 512, 1024};
    std::vector<int> iters = {20, 40, 80};

    std::cout << "Performance Benchmark (ms/step)\n";
    std::cout << "Steps/run: " << base.steps << ", Warmup: " << warmup << ", Measured runs: " << runs << "\n\n";
    std::cout << std::left << std::setw(12) << "Grid" << std::setw(10) << "Iters" << std::setw(16) << "CPU/step"
              << std::setw(20) << "GPU-kernel/step" << std::setw(18) << "GPU-e2e/step" << "Speedup(CPU/kernel)\n";

    for (const int n : sizes) {
        for (const int it : iters) {
            SimConfig cfg = base;
            cfg.width = n;
            cfg.height = n;
            cfg.pressure_iters = it;

            CpuFluid initial(cfg.width, cfg.height);
            initial.initialize_deterministic();

            CpuFluid cpu_state = initial;
            for (int i = 0; i < warmup; ++i) {
                for (int s = 0; s < cfg.steps; ++s) {
                    cpu_state.step(cfg);
                }
            }

            const auto cpu_start = std::chrono::high_resolution_clock::now();
            for (int i = 0; i < runs; ++i) {
                cpu_state = initial;
                for (int s = 0; s < cfg.steps; ++s) {
                    cpu_state.step(cfg);
                }
            }
            const auto cpu_stop = std::chrono::high_resolution_clock::now();
            const double cpu_total_ms =
                std::chrono::duration<double, std::milli>(cpu_stop - cpu_start).count();
            const double cpu_ms_per_step = cpu_total_ms / static_cast<double>(runs * cfg.steps);

            GpuFluid gpu(cfg.width, cfg.height);
            gpu.load_from_cpu(initial);
            const float gpu_kernel_ms = gpu.time_steps_kernel_ms(cfg, warmup, runs);

            CpuFluid out(cfg.width, cfg.height);
            const float gpu_e2e_ms = gpu.time_steps_end_to_end_ms(initial, out, cfg, warmup, runs);

            const double speedup = cpu_ms_per_step / static_cast<double>(gpu_kernel_ms);
            std::cout << std::left << std::setw(12) << (std::to_string(n) + "x" + std::to_string(n))
                      << std::setw(10) << it << std::setw(16) << static_cast<float>(cpu_ms_per_step)
                      << std::setw(20) << gpu_kernel_ms << std::setw(18) << gpu_e2e_ms << static_cast<float>(speedup) << "\n";
        }
    }
}

void run_visualize(int argc, char** argv) {
    SimConfig cfg;
    bool vsync = true;
    bool use_gpu_backend = true;
    int steps_per_frame = 1;
    int cpu_min_step_ms = 0;
    RenderField field = RenderField::Dye;
    parse_common_args(argc, argv, cfg, 2);

    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--vsync" && i + 1 < argc) {
            vsync = arg_to_int(argv[++i]) != 0;
        } else if (a == "--backend" && i + 1 < argc) {
            const std::string b = argv[++i];
            if (b == "gpu") {
                use_gpu_backend = true;
            } else if (b == "cpu") {
                use_gpu_backend = false;
            } else {
                throw std::runtime_error("Unknown --backend value (use gpu or cpu)");
            }
        } else if (a == "--cpu-min-step-ms" && i + 1 < argc) {
            cpu_min_step_ms = std::max(0, arg_to_int(argv[++i]));
        } else if (a == "--steps-per-frame" && i + 1 < argc) {
            steps_per_frame = std::max(1, arg_to_int(argv[++i]));
        } else if (a == "--field" && i + 1 < argc) {
            const std::string f = argv[++i];
            if (f == "velocity") {
                field = RenderField::Velocity;
            } else if (f == "pressure") {
                field = RenderField::Pressure;
            } else if (f == "divergence") {
                field = RenderField::Divergence;
            } else if (f == "dye") {
                field = RenderField::Dye;
            }
        }
    }

    CpuFluid cpu(cfg.width, cfg.height);
    cpu.initialize_deterministic();

    GpuFluid gpu(cfg.width, cfg.height);
    if (use_gpu_backend) {
        gpu.load_from_cpu(cpu);
    }

    VisualizeGL vis(cfg.width, cfg.height, vsync, use_gpu_backend);
    vis.set_title(use_gpu_backend ? "Fluid Visualization (GPU)" : "Fluid Visualization (CPU)");

    bool paused = false;
    bool prev_space = false;
    bool prev_1 = false;
    bool prev_2 = false;
    bool prev_3 = false;
    bool prev_4 = false;
    bool prev_left = false;
    double prev_mx = 0.0;
    double prev_my = 0.0;

    auto t_last = std::chrono::high_resolution_clock::now();
    int frame_count = 0;
    while (!vis.should_close()) {
        vis.poll_events();
        if (glfwGetKey(vis.window(), GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            break;
        }

        const bool space = glfwGetKey(vis.window(), GLFW_KEY_SPACE) == GLFW_PRESS;
        if (space && !prev_space) {
            paused = !paused;
        }
        prev_space = space;

        const bool k1 = glfwGetKey(vis.window(), GLFW_KEY_1) == GLFW_PRESS;
        const bool k2 = glfwGetKey(vis.window(), GLFW_KEY_2) == GLFW_PRESS;
        const bool k3 = glfwGetKey(vis.window(), GLFW_KEY_3) == GLFW_PRESS;
        const bool k4 = glfwGetKey(vis.window(), GLFW_KEY_4) == GLFW_PRESS;
        if (k1 && !prev_1) {
            field = RenderField::Velocity;
        }
        if (k2 && !prev_2) {
            field = RenderField::Pressure;
        }
        if (k3 && !prev_3) {
            field = RenderField::Divergence;
        }
        if (k4 && !prev_4) {
            field = RenderField::Dye;
        }
        prev_1 = k1;
        prev_2 = k2;
        prev_3 = k3;
        prev_4 = k4;

        double mx = 0.0;
        double my = 0.0;
        glfwGetCursorPos(vis.window(), &mx, &my);
        int fb_w = cfg.width;
        int fb_h = cfg.height;
        glfwGetFramebufferSize(vis.window(), &fb_w, &fb_h);
        const bool left_down = glfwGetMouseButton(vis.window(), GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        if (left_down) {
            const float nx = clampf(static_cast<float>(mx / static_cast<double>(std::max(1, fb_w))), 0.0f, 1.0f);
            const float ny = clampf(1.0f - static_cast<float>(my / static_cast<double>(std::max(1, fb_h))), 0.0f, 1.0f);
            if (prev_left) {
                const float dmx = static_cast<float>(mx - prev_mx);
                const float dmy = static_cast<float>(my - prev_my);
                // Screen Y grows downward; flip sign for fluid coordinates.
                if (use_gpu_backend) {
                    gpu.add_mouse_interaction(nx, ny, dmx, -dmy, 0.05f, 0.03f, 0.35f);
                } else {
                    cpu.add_mouse_interaction(nx, ny, dmx, -dmy, 0.05f, 0.03f, 0.35f);
                }
            } else if (use_gpu_backend) {
                gpu.add_mouse_interaction(nx, ny, 0.0f, 0.0f, 0.06f, 0.0f, 0.9f);
            } else {
                cpu.add_mouse_interaction(nx, ny, 0.0f, 0.0f, 0.06f, 0.0f, 0.9f);
            }
        }
        prev_left = left_down;
        prev_mx = mx;
        prev_my = my;

        if (!paused) {
            const auto step_start = std::chrono::high_resolution_clock::now();
            for (int s = 0; s < steps_per_frame; ++s) {
                if (use_gpu_backend) {
                    gpu.step(cfg);
                } else {
                    cpu.step(cfg);
                }
            }
            if (!use_gpu_backend && cpu_min_step_ms > 0) {
                const auto step_end = std::chrono::high_resolution_clock::now();
                const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(step_end - step_start).count();
                if (elapsed_ms < cpu_min_step_ms) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(cpu_min_step_ms - elapsed_ms));
                }
            }
        }

        if (use_gpu_backend) {
            vis.render(gpu, field);
        } else {
            vis.render(cpu, field);
        }

        ++frame_count;
        const auto t_now = std::chrono::high_resolution_clock::now();
        const double sec = std::chrono::duration<double>(t_now - t_last).count();
        if (sec >= 1.0) {
            std::cout << (use_gpu_backend ? "GPU" : "CPU") << " FPS: " << static_cast<double>(frame_count) / sec
                      << (paused ? " (paused)" : "") << "\n";
            frame_count = 0;
            t_last = t_now;
        }
    }
}

void print_usage() {
    std::cout << "Usage:\n";
    std::cout << "  ./fluid_sim verify [--width N --height N --steps N --iters N --dt F --dx F --force-amplitude F --force-sigma F]\n";
    std::cout << "  ./fluid_sim bench  [--steps N --dt F --dx F --force-amplitude F --force-sigma F --warmup N --runs N]\n";
    std::cout << "  ./fluid_sim visualize [--backend gpu|cpu --width N --height N --iters N --diffusion-iters N --viscosity F --dt F --dx F --force-amplitude F --force-sigma F --vsync 0|1 --steps-per-frame N --cpu-min-step-ms N --field velocity|pressure|divergence|dye]\n";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc < 2) {
            print_usage();
            return 0;
        }

        const std::string mode = argv[1];
        if (mode == "verify") {
            run_verify(argc, argv);
            return 0;
        }
        if (mode == "bench") {
            run_bench(argc, argv);
            return 0;
        }
        if (mode == "visualize") {
            run_visualize(argc, argv);
            return 0;
        }

        print_usage();
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 2;
    }
}

