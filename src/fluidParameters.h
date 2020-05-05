#ifndef FLUID_PARAMETERS_H
#define FLUID_PARAMETERS_H

#include "real.h"
#include "vector3R.h"

struct FluidParameters {
  FluidParameters()=default;
  FluidParameters(const FluidParameters& other)=default;
  FluidParameters(
    REAL density,
    REAL particle_mass,
    REAL damping,
    REAL h,
    REAL epsilon,
    REAL n,
    REAL k,
    REAL c,
    int solverIterations = 4,
    REAL particleRadius = 0.02)
      : density(density), particle_mass(particle_mass), damping(damping),
        h(h), epsilon(epsilon), n(n), k(k), c(c),
        solverIterations(solverIterations), particleRadius(particleRadius) {}
  ~FluidParameters() {}

  // Global simulation parameters
  Vector3R external_forces;
  REAL damping;
  REAL particle_mass;
  REAL h;
  REAL epsilon;
  REAL n;
  REAL k;
  REAL c;
  // unit: kg/m^3
  REAL density;
  int solverIterations;

  // render parameters

  // unit: m
  REAL particleRadius;
};

#endif