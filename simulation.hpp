#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <vector>
#include <cassert>

#include "cloth_mesh.hpp"
#include "math.hpp"
#include "constraint.hpp"
#include "misc.hpp"

using namespace std;

/* This is based on the paper "Fast Simulation of Mass Spring Systems"
 * As far as possible, the variables are named the same as in the  paper
 *
 * [1] Fast Simulation of Mass Spring Systems */

class Simulation {
 private:
  double h; /* time step */

  ClothMesh *mesh;
  vector<Mesh *> obstacle_meshes; /* TODO(ish): this is a
                                   * Sphere reference for now but
                                   * should be its own Mesh
                                   * structure */

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
  double stiffness_pin;
  double stiffness_bending;
  double damping_coefficient;
  Vec3 wind_direction;
  double wind_strength;
  double turbulence;
  double cloth_thickness;

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

  bool checkProximity(ClothNode *n, Face *f, Vec3 &r_bary_coords);
  bool findImpulse(const Vec3 &x1,
                   const Vec3 &x2,
                   const Vec3 &x3,
                   const Vec3 &x4,
                   const Vec3 &bary_coords,
                   const Vec3 &normal,
                   double v_n,
                   double mass,
                   double &r_impulse);
  void applyRepulsion(ClothNode *n, Face *f, const Vec3 &bary_coords);
  bool checkProximity(ClothFace *f1, Face *f2);
  void solveCollisions(Mesh *ob_mesh);
  void solveCollisions();

  void setConstraints();
  void clearConstraints();

  void dampVelocity();

  void applyTransformations();
  void unapplyTransformations();

 public:
  Simulation(ClothMesh *mesh) : mesh(mesh)
  {
    h = 0.03333d * 0.2;
    stiffness_stretch = 80.0d * 10;
    stiffness_pin = 120.0d * 10;
    stiffness_bending = 20.0d * 10;
    gravity_constant = 9.8d;
    damping_coefficient = 0.001;
    iterations_per_frame = 10;
    wind_direction = normalize(Vec3(1.0, 0, 0));
    wind_strength = 5.0d;
    turbulence = 5.0d;
    cloth_thickness = 0.001d;

    obstacle_meshes.clear();

    reset();
  }

  ~Simulation()
  {
    clearConstraints();
  }

  void update();
  void reset();

  bool tryToTogglePinConstraint(const Vec3 &p0,
                                const Vec3 &dir); /* this creates a new pin constraint if there
                                                   * isn't one, otherwise switches it off */
  void addPinConstraint(int index);               /* adds pin constraint at the
                                                   * ClothNode index */

  void drawConstraints(glm::mat4 &projection,
                       glm::mat4 &view,
                       bool draw_stretch,
                       bool draw_bending);

  void addObstacleMesh(Mesh *ob_mesh)
  {
    obstacle_meshes.push_back(ob_mesh);
  }
};

#endif
