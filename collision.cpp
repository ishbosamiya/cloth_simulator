#include "collision.hpp"

Collision::Collision(Simulation *simulation)
{
  this->simulation = simulation;
  /* TODO(ish): Currently timestep is not adaptive not allows
   * substepping */
  this->collision_timestep = this->simulation->h;
  buildBVH();
}

Collision::~Collision()
{
  deleteBVH();
}

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
  /* TODO(ish): calculate the impulse with respect to friction */

  /* Currently, since the obstacle doesn't have a velocity term, we
   * need to consider this as Vec3(0), otherwise the actual value
   * would be
   * Vec3 v_rel = cloth_node->v - interp(face->v[0]->node->v,
   * face->v[1]->node->v, face->v[2]->node->v, bary_coords); */
  Vec3 v_rel = cloth_node->v - Vec3(0);
  double vn = dot(face->n, v_rel);
  /* If vn < 0 then the node and the face are approaching each other */
  if (vn > 0) {
    return;
  }
  Vec3 &x1 = face->v[0]->node->x;
  Vec3 &x2 = face->v[1]->node->x;
  Vec3 &x3 = face->v[2]->node->x;
  Vec3 &x4 = cloth_node->x0;
  double d = simulation->cloth_thickness - dot(x4 - interp(x1, x2, x3, bary_coords), face->n);

  double I = -min(collision_timestep * simulation->stiffness_stretch * d,
                  cloth_node->mass * ((0.1 * d / collision_timestep) - vn));

  /* I_bar is the adjusted impulse, section 7.1 from
   * "Robust Treatment of Collisions, Contact, and Friction for Cloth
   * Animation" */
  double I_bar = 2.0 * I / (1 + norm2(bary_coords));
  cloth_node->impulse += face->n * I_bar;
  cloth_node->impulse_count++;
}

bool Collision::checkProximity(ClothNode *cloth_node, Face *face, Vec3 &r_bary_coords)
{
  /* Currently supports only static obstacle mesh */
  /* Following "Robust Treatment of Collisions, Contact, and Friction
   * for Cloth Animation"'s styling */
  Vec3 &x1 = face->v[0]->node->x;
  Vec3 &x2 = face->v[1]->node->x;
  Vec3 &x3 = face->v[2]->node->x;
  Vec3 &x4 = cloth_node->x0;
  Vec3 x43 = x4 - x3;

  /* Point x4 should be within simulation->cloth_thickness distance from the
   * plane of the face */
  if (fabs(dot(x43, face->n)) > simulation->cloth_thickness) {
    return false;
  }

  Vec3 x13 = x1 - x3;
  Vec3 x23 = x2 - x3;

  EigenMat2 mat;
  mat << dot(x13, x13), dot(x13, x23), dot(x13, x23), dot(x23, x23);
  EigenVec2 vec;
  vec << dot(x13, x43), dot(x23, x43);
  EigenVec2 w = mat.colPivHouseholderQr().solve(vec);

  /* Characteristic length of triangle is square root of the area of
   * the triangle */
  double delta = simulation->cloth_thickness / sqrt(0.5 * norm((normal(x1, x2, x3))));
  r_bary_coords[0] = w[0];
  r_bary_coords[1] = w[1];
  r_bary_coords[2] = 1.0 - (w[0] + w[1]);

  /* barycentric coordinates must be within [-delta, 1 + delta] */
  for (int i = 0; i < 3; i++) {
    if (r_bary_coords[i] < -delta) {
      return false;
    }
    if (r_bary_coords[i] > 1 + delta) {
      return false;
    }
  }

  return true;
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
  if (overlap_size == 0) {
    return;
  }

  setImpulseToZero(cloth_mesh);

  for (int i = 0; i < overlap_size; i++) {
    int cloth_mesh_index = overlap[i].indexA;
    int obstacle_mesh_index = overlap[i].indexB;

    ClothFace *cloth_face = static_cast<ClothFace *>(cloth_mesh->faces[cloth_mesh_index]);
    Face *obstacle_face = obstacle_mesh->faces[obstacle_mesh_index];

    checkProximityAndCalculateImpulse(cloth_face, obstacle_face);
  }
  /* TEMP */
  {
    int num_nodes = cloth_mesh->nodes.size();
    int count = 0;
    for (int i = 0; i < num_nodes; i++) {
      ClothNode *node = static_cast<ClothNode *>(cloth_mesh->nodes[i]);
      if (node->impulse_count == 0) {
        continue;
      }

      node->v = node->v - (node->impulse / (node->impulse_count * node->mass));
      node->x = node->x0 + (collision_timestep * node->v);
      count++;
    }
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
    obstacle_mesh->updateFaceNormals();

    solveCollision(cloth_mesh, obstacle_mesh);
  }
}
