#include "collision.hpp"

void Collision::buildBVH()
{
  simulation->mesh->buildBVH();
  int obstacle_meshes_size = simulation->obstacle_meshes.size();
  for (int i = 0; i < obstacle_meshes_size; i++) {
    simulation->obstacle_meshes[i]->buildBVH();
  }
}

void Collision::deleteBVH()
{
  simulation->mesh->deleteBVH();
  int obstacle_meshes_size = simulation->obstacle_meshes.size();
  for (int i = 0; i < obstacle_meshes_size; i++) {
    simulation->obstacle_meshes[i]->deleteBVH();
  }
}

void Collision::calculateImpulse(ClothNode *cloth_node, Face *face, Vec3 bary_coords)
{
  /* TODO(ish): calculate the impulse with respect to friction,
   * repulsion, storing it in impulse and incrementing impulse_count*/
}

bool Collision::checkProximity(ClothNode *cloth_node, Face *face, Vec3 &r_bary_coords)
{
  /* TODO(ish): check for proximity of cloth_node->x0 with respect to
   * face, currently static obstacle mesh, and also compute the
   * respective barycentric coordinates */
}

void Collision::checkProximityAndCalculateImpulse(ClothFace *cloth_face, Face *obstacle_face)
{
  for (int i = 0; i < 3; i++) {
    ClothNode *node = static_cast<ClothNode *>(cloth_face->v[i]->node);

    Vec3 bary_coords;
    if (checkProximity(node, obstacle_face, bary_coords)) {
      /* Impulse will be calculated and stored in node->impulse,
       * node->impulse_count */
      calculateImpulse(node, obstacle_face, bary_coords);
    }
  }
}

static void setImpulseToZero(ClothMesh *cloth_mesh)
{
  int nodes_size = cloth_mesh->nodes.size();
  for (int i = 0; i < nodes_size; i++) {
    ClothNode *node = static_cast<ClothNode *>(cloth_mesh->nodes[i]);

    node->impulse_count = 0;
    node->impulse = Vec3(0.0d);
  }
}

void Collision::solveCollision(ClothMesh *cloth_mesh, Mesh *obstacle_mesh)
{
  unsigned int overlap_size = 0;
  BVHTreeOverlap *overlap = BVHTree_overlap(
      cloth_mesh->bvh, obstacle_mesh->bvh, &overlap_size, NULL, NULL);

  setImpulseToZero(cloth_mesh);

  for (int i = 0; i < overlap_size; i++) {
    int cloth_mesh_index = overlap[i].indexA;
    int obstacle_mesh_index = overlap[i].indexB;

    ClothFace *cloth_face = static_cast<ClothFace *>(cloth_mesh->faces[cloth_mesh_index]);
    Face *obstacle_face = obstacle_mesh->faces[obstacle_mesh_index];

    checkProximityAndCalculateImpulse(cloth_face, obstacle_face);
  }

  /* TODO(ish): now that impulse in stored in the nodes of the
   * cloth_mesh, we must apply impulse to get node->tv, which is then
   * used for RIZ, etc. */
  /* TODO(ish): Later on when the obstacle mesh is also affected by the
   * cloth/during self collision, the barycoords need to be carried
   * over to this part as well */
}

void Collision::solveCollision(bool rebuild_bvh)
{
  if (rebuild_bvh) {
    deleteBVH();
    buildBVH();
  }

  /* TODO(ish): add collision steps, rigid impact zones */
  /* TODO(ish): self collisions */
  /* TODO(ish): add support for moving obstacle_meshes */
  ClothMesh *cloth_mesh = simulation->mesh;
  /* Need to update BVH */
  cloth_mesh->updateBVH();
  vector<Mesh *> &obstacle_meshes = simulation->obstacle_meshes;
  int obstacle_meshes_size = obstacle_meshes.size();
  for (int i = 0; i < obstacle_meshes_size; i++) {
    Mesh *obstacle_mesh = obstacle_meshes[i];
    /* Need to update BVH */
    obstacle_mesh->updateBVH();

    solveCollision(cloth_mesh, obstacle_mesh);
  }
}
