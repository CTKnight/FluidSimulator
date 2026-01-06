#pragma once

#include <cstddef>
#include <vector>

namespace fluid {

int core_version();

struct Params {
  float dt = 1.0f / 60.0f;
  enum class Backend {
    Cpu,
    Cuda,
  };
  Backend backend = Backend::Cpu;
};

struct State {
  std::vector<float> pos_x;
  std::vector<float> pos_y;
  std::vector<float> pos_z;
  std::vector<float> vel_x;
  std::vector<float> vel_y;
  std::vector<float> vel_z;
  float time = 0.0f;

  std::size_t size() const { return pos_x.size(); }
};

State make_state(std::size_t particle_count);
void step(const Params& params, State& state);

}  // namespace fluid
