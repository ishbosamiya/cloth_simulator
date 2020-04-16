#ifndef COLLISION_HPP
#define COLLISION_HPP

#include <vector>

#include "simulation.hpp"
#include "bvh.hpp"

using namespace std;

class Simulation;

class Collision {
 private:
  Simulation *simulation;
  void buildBVH();
  void deleteBVH();

  void solveCollision(ClothMesh *cloth_mesh, Mesh *obstacle_mesh);

 public:
  Collision(Simulation *simulation)
  {
    this->simulation = simulation;
    buildBVH();
  }

  /* rebuild_bvh is necessary when the mesh structure has changed, eg:
   * adaptive remeshing of the cloth */
  void solveCollision(bool rebuild_bvh = false);

  ~Collision()
  {
    deleteBVH();
  }
};

#endif
