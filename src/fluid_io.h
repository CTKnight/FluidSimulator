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
  os << "<?xml version=\"1.0\"?>\n";
  os << "<VTKFile type=\"PolyData\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
  os << "  <PolyData>\n";
  os << "    <Piece NumberOfPoints=\"" << particles.size()
     << "\" NumberOfVerts=\"" << particles.size()
     << "\" NumberOfLines=\"0\" NumberOfStrips=\"0\" NumberOfPolys=\"0\">\n";
  os << "      <Points>\n";
  os << "        <DataArray type=\"Float32\" NumberOfComponents=\"3\" format=\"ascii\">\n";
  os << std::setprecision(9) << std::fixed;
  for (const auto &p: particles) {
    os << "          " << p << "\n";
  }
  os << "        </DataArray>\n";
  os << "      </Points>\n";
  os << "      <Verts>\n";
  os << "        <DataArray type=\"Int32\" Name=\"connectivity\" format=\"ascii\">\n";
  for (std::size_t i = 0; i < particles.size(); ++i) {
    os << "          " << i << "\n";
  }
  os << "        </DataArray>\n";
  os << "        <DataArray type=\"Int32\" Name=\"offsets\" format=\"ascii\">\n";
  for (std::size_t i = 0; i < particles.size(); ++i) {
    os << "          " << (i + 1) << "\n";
  }
  os << "        </DataArray>\n";
  os << "      </Verts>\n";
  os << "    </Piece>\n";
  os << "  </PolyData>\n";
  os << "</VTKFile>\n";
  return os;
}

#ifdef BUILD_CUDA
std::ostream& operator<<(std::ostream& os, const Fluid_cuda& fluid) {
  const auto &particles = fluid.getParticlePositions();
  os << "<?xml version=\"1.0\"?>\n";
  os << "<VTKFile type=\"PolyData\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
  os << "  <PolyData>\n";
  os << "    <Piece NumberOfPoints=\"" << particles.size()
     << "\" NumberOfVerts=\"" << particles.size()
     << "\" NumberOfLines=\"0\" NumberOfStrips=\"0\" NumberOfPolys=\"0\">\n";
  os << "      <Points>\n";
  os << "        <DataArray type=\"Float32\" NumberOfComponents=\"3\" format=\"ascii\">\n";
  os << std::setprecision(9) << std::fixed;
  for (const auto &p: particles) {
    os << "          " << p << "\n";
  }
  os << "        </DataArray>\n";
  os << "      </Points>\n";
  os << "      <Verts>\n";
  os << "        <DataArray type=\"Int32\" Name=\"connectivity\" format=\"ascii\">\n";
  for (std::size_t i = 0; i < particles.size(); ++i) {
    os << "          " << i << "\n";
  }
  os << "        </DataArray>\n";
  os << "        <DataArray type=\"Int32\" Name=\"offsets\" format=\"ascii\">\n";
  for (std::size_t i = 0; i < particles.size(); ++i) {
    os << "          " << (i + 1) << "\n";
  }
  os << "        </DataArray>\n";
  os << "      </Verts>\n";
  os << "    </Piece>\n";
  os << "  </PolyData>\n";
  os << "</VTKFile>\n";
  return os;
}
#endif

#endif
