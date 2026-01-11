#pragma once

#include "fluid/core.h"

namespace fluid {

int cuda_version();
void cuda_step(const Params& params, State& state);

}  // namespace fluid
