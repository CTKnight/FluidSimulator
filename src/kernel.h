
#include "real.h"
#include "vector3R.h"
#include "cuda/cudaUtils.h"

#define PI (3.14159265358979323)

CUDA_CALLABLE inline REAL W_poly6(const Vector3R &r, REAL h) {
  const auto r2 = r.norm2();
  const auto h2 = pow(h, 2);
  if (r2 > h2) {
    return 0;
  }
  return 315 / (64 * PI * pow(h, 9)) * pow(h2 - r2, 3);
}

// https://www.wolframalpha.com/input/?i=gradient+15%2F%28PI*h%5E6%29*%28h-x%29%5E3
CUDA_CALLABLE inline REAL W_spiky_gradient(const Vector3R &r_vec, REAL h) {
  const auto r = r_vec.norm();
  if (r > h) {
    return 0;
  }
  return -45 / (PI * pow(h, 6)) * pow(h - r, 2);
}