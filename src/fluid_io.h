#ifndef FLUID_IO_H
#define FLUID_IO_H
#include <iomanip>
#include <iostream>
#include <string>

#include "real.h"
#include "fluid.h"

#ifdef BUILD_CUDA
#include "cuda/cudaFluid.cuh"
#endif

std::istream& operator>>(std::istream& is, REAL3& v) {
  is >> v[0] >> v[1] >> v[2];
  return is;
}

std::ostream& operator<<(std::ostream& os, const REAL3& v) {
  os << v[0] << ' ' << v[1] << ' ' << v[2];
  return os;
}

std::istream& operator>>(std::istream& is, Fluid& fluid) {
  string placeholder;
  int n;
  auto &particles = fluid.getParticlePositions();
  for (int i = 0; i < 4; i++) {
    std::getline(is, placeholder);
  }
  is >> placeholder >> n >> placeholder;
  std::getline(is, placeholder);
  particles.resize(n);
  for (int i = 0; i < n; i++) {
    is >> particles[i];
  }
  return is;
}

std::ostream& operator<<(std::ostream& os, const Fluid& fluid) {
  const auto &particles = fluid.getParticlePositions();
  // VTK headers
  os << "# vtk DataFile Version 3.0\n";
  os << "vtk output\n";
  os << "ASCII\n";
  os << "DATASET UNSTRUCTURED_GRID\n";
  os << "POINTS " << particles.size() << " float\n";
  os << std::setprecision(5) << std::fixed;
  for (const auto &p: particles) {
    os << p << "\n";
  }
  return os;
}

#ifdef BUILD_CUDA
std::ostream& operator<<(std::ostream& os, const Fluid_cuda& fluid) {
  const auto &particles = fluid.getParticlePositions();
  // VTK headers
  os << "# vtk DataFile Version 3.0\n";
  os << "vtk output\n";
  os << "ASCII\n";
  os << "DATASET UNSTRUCTURED_GRID\n";
  os << "POINTS " << particles.size() << " float\n";
  os << std::setprecision(5) << std::fixed;
  for (const auto &p: particles) {
    os << p << "\n";
  }
  return os;
}
#endif

#endif