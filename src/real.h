#ifndef REAL_H
#define REAL_H

#include <CompactNSearch>

#ifdef USE_DOUBLE
using REAL = double;
#else
using REAL = float;
#endif

#endif