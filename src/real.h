#ifndef REAL_H
#define REAL_H

#include <array>
#include <CompactNSearch>

#ifdef USE_DOUBLE
using REAL = double;
#else
using REAL = float;
#endif

using REAL3 = std::array<REAL, 3>;

#endif