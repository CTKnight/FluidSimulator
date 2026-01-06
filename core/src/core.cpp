#include "fluid/core.h"

namespace fluid {

int core_version() {
  return 1;
}

State make_state(std::size_t particle_count) {
  State state;
  state.pos_x.resize(particle_count, 0.0f);
  state.pos_y.resize(particle_count, 0.0f);
  state.pos_z.resize(particle_count, 0.0f);
  state.vel_x.resize(particle_count, 0.0f);
  state.vel_y.resize(particle_count, 0.0f);
  state.vel_z.resize(particle_count, 0.0f);
  return state;
}

void step(const Params& params, State& state) {
  const std::size_t count = state.size();
  for (std::size_t i = 0; i < count; ++i) {
    state.pos_x[i] += state.vel_x[i] * params.dt;
    state.pos_y[i] += state.vel_y[i] * params.dt;
    state.pos_z[i] += state.vel_z[i] * params.dt;
  }
  state.time += params.dt;
}

}  // namespace fluid
