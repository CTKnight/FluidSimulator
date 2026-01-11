#include "fluid/init.h"

#include <cmath>

namespace fluid {

namespace {

float default_spacing(const Params& params) {
  if (params.density <= 0.0f || params.particle_mass <= 0.0f) {
    return 0.0f;
  }
  const float volume_per_particle = params.particle_mass / params.density;
  return std::cbrt(volume_per_particle);
}

std::size_t count_from_extent(float extent, float spacing) {
  if (extent <= 0.0f || spacing <= 0.0f) {
    return 0;
  }
  const std::size_t count =
      static_cast<std::size_t>(std::floor(extent / spacing));
  return count > 0 ? count : 1;
}

}  // namespace

void init_block(const Params& params,
                State& state,
                std::size_t nx,
                std::size_t ny,
                std::size_t nz,
                float origin_x,
                float origin_y,
                float origin_z,
                float spacing) {
  const std::size_t count = nx * ny * nz;
  state.pos_x.assign(count, 0.0f);
  state.pos_y.assign(count, 0.0f);
  state.pos_z.assign(count, 0.0f);
  state.vel_x.assign(count, 0.0f);
  state.vel_y.assign(count, 0.0f);
  state.vel_z.assign(count, 0.0f);
  state.time = 0.0f;
  state.cpu.resize(count);

  float step = spacing;
  if (step <= 0.0f) {
    step = default_spacing(params);
  }

  std::size_t index = 0;
  for (std::size_t z = 0; z < nz; ++z) {
    for (std::size_t y = 0; y < ny; ++y) {
      for (std::size_t x = 0; x < nx; ++x) {
        state.pos_x[index] = origin_x + static_cast<float>(x) * step;
        state.pos_y[index] = origin_y + static_cast<float>(y) * step;
        state.pos_z[index] = origin_z + static_cast<float>(z) * step;
        index++;
      }
    }
  }
}

void init_planes(Params& params, float box_x, float box_y, float box_z) {
  params.planes.clear();
  params.planes.add_normalized(1.0f, 0.0f, 0.0f, 0.0f);
  params.planes.add_normalized(-1.0f, 0.0f, 0.0f, -box_x);
  params.planes.add_normalized(0.0f, 1.0f, 0.0f, 0.0f);
  params.planes.add_normalized(0.0f, -1.0f, 0.0f, -box_y);
  params.planes.add_normalized(0.0f, 0.0f, 1.0f, 0.0f);
  params.planes.add_normalized(0.0f, 0.0f, -1.0f, -box_z);
}

void init_test_scene(Params& params, State& state) {
  const float box_x = 1.0f;
  const float box_y = 1.0f;
  const float box_z = 1.0f;
  init_planes(params, box_x, box_y, box_z);

  float spacing = 0.0f;
  if (params.particle_radius > 0.0f) {
    spacing = params.particle_radius * 2.0f;
  }
  if (spacing <= 0.0f) {
    spacing = default_spacing(params);
  }
  if (spacing <= 0.0f) {
    spacing = 0.02f;
  }

  if (params.density > 0.0f) {
    params.particle_mass = params.density * spacing * spacing * spacing;
  }
  params.h = 2.5f * spacing;

  const float block_x = box_x * 0.4f;
  const float block_y = box_y * 0.4f;
  const float block_z = box_z * 0.4f;

  const std::size_t nx = count_from_extent(block_x, spacing);
  const std::size_t ny = count_from_extent(block_y, spacing);
  const std::size_t nz = count_from_extent(block_z, spacing);

  const float block_size_x = static_cast<float>(nx) * spacing;
  const float block_size_z = static_cast<float>(nz) * spacing;
  const float origin_x = 0.5f * (box_x - block_size_x);
  const float origin_y = 0.5f * box_y;
  const float origin_z = 0.5f * (box_z - block_size_z);

  init_block(params, state, nx, ny, nz, origin_x, origin_y, origin_z, spacing);
}

}  // namespace fluid
