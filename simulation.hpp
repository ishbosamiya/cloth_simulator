#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <vector>

#include "cloth_mesh.hpp"
#include "math.hpp"
#include "constraint.hpp"

using namespace std;

/* This is based on the paper "Fast Simulation of Mass Spring Systems"
 * As far as possible, the variables are named the same as in the  paper
 *
 * [1] Fast Simulation of Mass Spring Systems */

class Simulation {
 private:
  float h; /* time step */

  ClothMesh *mesh;

  vector<Constraint *> constraints; /* Springs, pinning, etc. */

  EigenVecX inertia_y;
  EigenVecX external_forces;

  unsigned int iterations_per_frame;

  EigenSparseMatrix weighted_laplacian;                                     /* needed in [1] */
  EigenSparseMatrix j_matrix;                                               /* needed in [1] */
  Eigen::SimplicialLLT<EigenSparseMatrix, Eigen::Upper> prefactored_solver; /* needed in [1] */
};

#endif
