#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "fluid/cli.h"
#include "fluid/core.h"
#include "fluid/init.h"
#include "fluid/vtk_writer.h"

#ifdef FLUID_ENABLE_CUDA
#include "fluid/cuda.h"
#endif

int main(int argc, char** argv) {
  const std::vector<fluid::cli::OptionSpec> specs = {
      {"backend", fluid::cli::OptionType::Value,
       "Backend to use: cpu or cuda (if built)."},
      {"no-output", fluid::cli::OptionType::Flag,
       "Disable output for benchmarking."},
      {"debug-print", fluid::cli::OptionType::Flag,
       "Print a small CPU sanity snapshot after one step."},
      {"steps", fluid::cli::OptionType::Value,
       "Number of simulation steps to run."},
      {"scene", fluid::cli::OptionType::Value,
       "Scene JSON to load (legacy format)."},
      {"output-dir", fluid::cli::OptionType::Value,
       "Directory for VTK output (for ParaView)."},
      {"parity", fluid::cli::OptionType::Flag,
       "Run CPU vs CUDA parity check (no output)."},
  };

  const fluid::cli::ParseResult parsed =
      fluid::cli::parse_args(argc, argv, specs);
  if (!parsed.ok) {
    std::cerr << parsed.error << std::endl;
    std::cerr << fluid::cli::make_usage(argv[0], specs);
    return 1;
  }
  if (parsed.has("help")) {
    std::cout << fluid::cli::make_usage(argv[0], specs);
    return 0;
  }

  const std::string backend = parsed.value("backend", "cpu");
  const bool output_enabled = !parsed.has("no-output");
  const bool debug_print = parsed.has("debug-print");
  const std::string output_dir = parsed.value("output-dir", "output");
  const std::string scene_path = parsed.value("scene", "");
  const bool parity = parsed.has("parity");
  int steps = 1;
  try {
    steps = std::stoi(parsed.value("steps", "1"));
  } catch (const std::exception&) {
    std::cerr << "Invalid steps value." << std::endl;
    std::cerr << fluid::cli::make_usage(argv[0], specs);
    return 1;
  }
  if (steps < 1) {
    std::cerr << "Steps must be >= 1." << std::endl;
    return 1;
  }

  if (backend != "cpu" && backend != "cuda") {
    std::cerr << "Unsupported backend: " << backend << std::endl;
    std::cerr << fluid::cli::make_usage(argv[0], specs);
    return 1;
  }

  std::cout << "FluidSimulator rewrite scaffold" << std::endl;
  std::cout << "core_version=" << fluid::core_version() << std::endl;

  if (backend == "cuda") {
#ifdef FLUID_ENABLE_CUDA
    std::cout << "backend=cuda (stub)" << std::endl;
    std::cout << "cuda_version=" << fluid::cuda_version() << std::endl;
#else
    std::cerr << "CUDA backend requested but not built." << std::endl;
    return 1;
#endif
  } else {
    std::cout << "backend=cpu" << std::endl;
  }

  fluid::Params params;
  params.backend = (backend == "cuda")
                       ? fluid::Params::Backend::Cuda
                       : fluid::Params::Backend::Cpu;
  fluid::State state = fluid::make_state(0);
  if (!scene_path.empty()) {
    std::string error;
    if (!fluid::init_scene_from_json(scene_path, params, state, &error)) {
      std::cerr << "Failed to load scene: " << error << std::endl;
      return 1;
    }
  } else {
    fluid::init_test_scene(params, state);
  }
  if (parity) {
#ifdef FLUID_ENABLE_CUDA
    fluid::State cpu_state = state;
    fluid::State cuda_state = state;
    for (int step = 0; step < steps; ++step) {
      const auto cpu_start = std::chrono::steady_clock::now();
      fluid::step(params, cpu_state);
      const auto cpu_end = std::chrono::steady_clock::now();
      const auto cuda_start = std::chrono::steady_clock::now();
      fluid::cuda_step(params, cuda_state);
      const auto cuda_end = std::chrono::steady_clock::now();

      double max_abs_pos = 0.0;
      double sum_sq_pos = 0.0;
      double max_abs_vel = 0.0;
      double sum_sq_vel = 0.0;
      const std::size_t n = cpu_state.size();
      for (std::size_t i = 0; i < n; ++i) {
        const double dx = static_cast<double>(cpu_state.pos_x[i]) -
                          static_cast<double>(cuda_state.pos_x[i]);
        const double dy = static_cast<double>(cpu_state.pos_y[i]) -
                          static_cast<double>(cuda_state.pos_y[i]);
        const double dz = static_cast<double>(cpu_state.pos_z[i]) -
                          static_cast<double>(cuda_state.pos_z[i]);
        const double adx = std::abs(dx);
        const double ady = std::abs(dy);
        const double adz = std::abs(dz);
        max_abs_pos = std::max(max_abs_pos, adx);
        max_abs_pos = std::max(max_abs_pos, ady);
        max_abs_pos = std::max(max_abs_pos, adz);
        sum_sq_pos += dx * dx + dy * dy + dz * dz;

        const double dvx = static_cast<double>(cpu_state.vel_x[i]) -
                           static_cast<double>(cuda_state.vel_x[i]);
        const double dvy = static_cast<double>(cpu_state.vel_y[i]) -
                           static_cast<double>(cuda_state.vel_y[i]);
        const double dvz = static_cast<double>(cpu_state.vel_z[i]) -
                           static_cast<double>(cuda_state.vel_z[i]);
        const double advx = std::abs(dvx);
        const double advy = std::abs(dvy);
        const double advz = std::abs(dvz);
        max_abs_vel = std::max(max_abs_vel, advx);
        max_abs_vel = std::max(max_abs_vel, advy);
        max_abs_vel = std::max(max_abs_vel, advz);
        sum_sq_vel += dvx * dvx + dvy * dvy + dvz * dvz;
      }

      const double denom = (n > 0) ? (3.0 * static_cast<double>(n)) : 1.0;
      const double rmse_pos = std::sqrt(sum_sq_pos / denom);
      const double rmse_vel = std::sqrt(sum_sq_vel / denom);
      const double cpu_ms =
          std::chrono::duration<double, std::milli>(cpu_end - cpu_start)
              .count();
      const double cuda_ms =
          std::chrono::duration<double, std::milli>(cuda_end - cuda_start)
              .count();
      std::cout << "parity_step=" << (step + 1) << " cpu_ms=" << cpu_ms
                << " cuda_ms=" << cuda_ms << " max_abs_pos=" << max_abs_pos
                << " rmse_pos=" << rmse_pos << " max_abs_vel=" << max_abs_vel
                << " rmse_vel=" << rmse_vel << std::endl;
    }
    std::cout << "particle_count=" << cpu_state.size() << std::endl;
    std::cout << "time=" << cpu_state.time << std::endl;
    return 0;
#else
    std::cerr << "Parity mode requires CUDA backend." << std::endl;
    return 1;
#endif
  }
  fluid::VtkWriter vtk_writer(output_dir, "frame");
  fluid::PvdWriter pvd_writer(output_dir, "series");
  for (int step = 0; step < steps; ++step) {
    const auto step_start = std::chrono::steady_clock::now();
    if (backend == "cuda") {
#ifdef FLUID_ENABLE_CUDA
      fluid::cuda_step(params, state);
#else
      std::cerr << "CUDA backend requested but not built." << std::endl;
      return 1;
#endif
    } else {
      fluid::step(params, state);
    }
    const auto step_end = std::chrono::steady_clock::now();
    const double step_ms =
        std::chrono::duration<double, std::milli>(step_end - step_start)
            .count();
    if (output_enabled) {
      fluid::VtkFrameView frame;
      frame.pos_x = state.pos_x.data();
      frame.pos_y = state.pos_y.data();
      frame.pos_z = state.pos_z.data();
      frame.count = state.size();
      frame.time = state.time;
      const std::size_t frame_index = static_cast<std::size_t>(step);
      if (!vtk_writer.write_frame(frame, frame_index)) {
        std::cerr << "Failed to write VTK frame." << std::endl;
        return 1;
      }
      pvd_writer.add_frame(
          state.time,
          fluid::VtkWriter::make_frame_filename("frame", frame_index));
    }
    if (debug_print) {
      std::cout << "step_done=" << (step + 1) << " step_ms=" << step_ms
          << std::endl;
    }
  }
  std::cout << "particle_count=" << state.size() << std::endl;
  std::cout << "time=" << state.time << std::endl;
  std::cout << "output_enabled=" << (output_enabled ? "true" : "false")
            << std::endl;
  if (output_enabled && !pvd_writer.write()) {
    std::cerr << "Failed to write PVD index." << std::endl;
    return 1;
  }

  return 0;
}
