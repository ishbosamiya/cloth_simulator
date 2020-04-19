#ifndef COLLISION_HPP
#define COLLISION_HPP

/* How collision algorithm works
 * => Let node->x be position after Simulation step
 * => node->x0 be position before Simulation step (previously known
 * correct, collision free state)
 * => node->v be the velocity as calculated by the Simulation step
 * => node->impulse be the impulse calculated during the collision
 * response
 * => node->impulse_count be the count of the impulses that have been
 * added to node->impulse
 * => node->tv be the temporary velocity, useful for collision response
 *
 * Iterate over all the faces of the cloth mesh and the obstacle
 * meshes using a BVHTree to determine overlapping AABBs
 *
 * For the overlapping faces:
 *     For node in face1:
 *         If (proximity of node->x0 with node->v with face2):
 *             (This must be for face2's node->x0 itself)
 *             Apply collision response to add to node->impulse and
 *             node->impulse_count
 * (Maybe) Do the same for nodes in face2 with face1
 * For node in Mesh:
 *     Find node->tv using node->impulse and node->impulse_count
 * TOOO(ish): currently set node->v to node->tv, need to still
 *     implement solving cubic equaltion and RIZ
 */

#include <iostream>
#include <vector>

#include "simulation.hpp"
#include "bvh.hpp"
#include "math.hpp"
#include "mesh.hpp"
#include "cloth_mesh.hpp"

using namespace std;

class Simulation;

class Collision {
 private:
  Simulation *simulation;
  double collision_timestep;
  void buildBVH();
  void deleteBVH();

  void calculateImpulse(ClothNode *cloth_node,
                        Face *face,
                        Vec3 bary_coords,
                        double coeff_friction);
  bool checkProximity(ClothNode *cloth_node, Face *face, Vec3 &r_bary_coords);
  void checkProximityAndCalculateImpulse(ClothFace *cloth_face,
                                         Face *obstacle_face,
                                         double coeff_friction);
  void solveCollision(ClothMesh *cloth_mesh, Mesh *obstacle_mesh);

 public:
  Collision(Simulation *simulation);

  /* rebuild_bvh is necessary when the mesh structure has changed, eg:
   * adaptive remeshing of the cloth */
  void solveCollision(bool rebuild_bvh = false);

  ~Collision();
};

#endif