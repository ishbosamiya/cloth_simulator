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
  double h; /* time step */

  ClothMesh *mesh;

  vector<Constraint *> constraints; /* Springs, pinning, etc. */

  EigenVecX inertia_y;
  EigenVecX external_forces;

  unsigned int iterations_per_frame;

  EigenSparseMatrix weighted_laplacian;                                     /* needed in [1] */
  EigenSparseMatrix j_matrix;                                               /* needed in [1] */
  Eigen::SimplicialLLT<EigenSparseMatrix, Eigen::Upper> prefactored_solver; /* needed in [1] */

  /* parameters */
  double gravity_constant;
  double stiffness_stretch;

  void calculateInertiaY();
  void calculateExternalForces();

  void updatePosAndVelocity(const EigenVecX &new_pos);

  void evaluateJMatrix(EigenSparseMatrix &r_j);
  void evaluateDVector(const EigenVecX &x, EigenVecX &r_d);
  void setWeightedLaplacianMatrix();
  void setJMatrix();
  void factorizeDirectSolverLLT(
      const EigenSparseMatrix &a,
      Eigen::SimplicialLLT<EigenSparseMatrix, Eigen::Upper> &r_llt_solver);
  void prefactorize();

  void integrateLocalGlobalOneIteration(EigenVecX &r_x, bool run_prefactorization);
  void integrateOptimization();

  void setConstraints();
  void clearConstraints();

 public:
  Simulation(ClothMesh *mesh) : mesh(mesh)
  {
    h = 0.03333d;
    stiffness_stretch = 80.0d;
    gravity_constant = 9.8d;
    iterations_per_frame = 10;

    reset();
  }

  ~Simulation()
  {
    clearConstraints();
  }

  void update();
  void reset();
};

#endif
