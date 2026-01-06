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
  float density = 1000.0f;
  float particle_mass = 1.0f;
  float h = 0.1f;
  float epsilon = 1e-6f;
  float n = 4.0f;
  float k = 0.1f;
  float c = 0.01f;
  int solver_iterations = 4;
  float particle_radius = 0.02f;
  float neighbor_reserve_factor = 1.5f;
  struct Vec3 {
    float x = 0.0f;
    float y = -9.81f;
    float z = 0.0f;
  } external_forces;
};

struct CpuScratch {
  std::vector<float> pred_x;
  std::vector<float> pred_y;
  std::vector<float> pred_z;
  std::vector<float> delta_x;
  std::vector<float> delta_y;
  std::vector<float> delta_z;
  std::vector<float> lambda;
  std::vector<int> neighbor_indices;
  std::vector<int> neighbor_prefix_sum;

  void resize(std::size_t particle_count);
  void clear_neighbors();
};

struct State {
  std::vector<float> pos_x;
  std::vector<float> pos_y;
  std::vector<float> pos_z;
  std::vector<float> vel_x;
  std::vector<float> vel_y;
  std::vector<float> vel_z;
  CpuScratch cpu;
  float time = 0.0f;

  std::size_t size() const { return pos_x.size(); }
};

State make_state(std::size_t particle_count);
void step(const Params& params, State& state);

}  // namespace fluid
