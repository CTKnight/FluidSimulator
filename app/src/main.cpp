#include <chrono>
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
      {"output-dir", fluid::cli::OptionType::Value,
       "Directory for VTK output (for ParaView)."},
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
  params.external_forces.x = 0.0f;
  params.external_forces.y = 0.0f;
  params.external_forces.z = 0.0f;
  fluid::State state = fluid::make_state(0);
  fluid::init_test_scene(params, state);
  fluid::VtkWriter vtk_writer(output_dir, "frame");
  fluid::PvdWriter pvd_writer(output_dir, "series");
  for (int step = 0; step < steps; ++step) {
    const auto step_start = std::chrono::steady_clock::now();
    fluid::step(params, state);
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
