#pragma once

#include <cstddef>

#include "fluid/core.h"

namespace fluid {

void init_block(const Params& params,
                State& state,
                std::size_t nx,
                std::size_t ny,
                std::size_t nz,
                float origin_x,
                float origin_y,
                float origin_z,
                float spacing = 0.0f);

void init_planes(Params& params, float box_x, float box_y, float box_z);

void init_test_scene(Params& params, State& state);

}  // namespace fluid
