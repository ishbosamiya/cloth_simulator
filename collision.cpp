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

void Collision::solveCollision(ClothMesh *cloth_mesh, Mesh *obstacle_mesh)
{
}

void Collision::solveCollision(bool rebuild_bvh)
{
  if (rebuild_bvh) {
    deleteBVH();
    buildBVH();
  }

  /* TODO(ish): add collision steps, rigin impact zones */
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
