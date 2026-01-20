#pragma once

#include "fluid/core.h"

namespace fluid {

int cuda_version();
bool cuda_device_available(int* count, const char** error);
void cuda_step(const Params& params, State& state);

}  // namespace fluid
