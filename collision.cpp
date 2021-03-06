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
  simulation->mesh->buildBVH(simulation->cloth_thickness);
  int obstacle_meshes_size = simulation->obstacle_meshes.size();
  for (int i = 0; i < obstacle_meshes_size; i++) {
    simulation->obstacle_meshes[i]->buildBVH(simulation->cloth_thickness);
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

/* Returns true if impulse has been calculated */
bool Collision::calculateImpulse(ImpulseInfo &info, Vec3 &r_impulse)
{
  Vec3 v_rel = *(info.v4) - interp(*(info.v1), *(info.v2), *(info.v3), info.bary_coords);
  double vn = dot(*(info.n), v_rel);
  /* If vn < 0 then the node and the face are approaching each other */
  if (vn > 0) {
    return false;
  }
  double d = simulation->cloth_thickness -
             dot(*(info.x4) - interp(*(info.x1), *(info.x2), *(info.x3), info.bary_coords),
                 *(info.n));

  double I = -min(collision_timestep * simulation->stiffness_stretch * d,
                  *(info.mass) * ((0.1 * d / collision_timestep) - vn));

  /* Friction */
  Vec3 v_rel_t_pre = v_rel - *(info.n) * vn;
  Vec3 v_rel_t = max(1.0 - (*(info.coeff_friction) * vn / norm(v_rel_t_pre)), 0.0) * v_rel_t_pre;
  /* Impulse of friction with the correct direction */
  Vec3 I_f = (v_rel_t - v_rel_t_pre) * *(info.mass);

  /* I_bar is the adjusted impulse, section 7.1 from
   * "Robust Treatment of Collisions, Contact, and Friction for Cloth
   * Animation" */
  double I_bar = 2.0 * I / (1 + norm2(info.bary_coords));
  r_impulse += (*(info.n) * I_bar) + I_f;

  return true;
}

/* Checks proximity of x4 with face formed by x1, x2, x3 where n is
 * the normal of the face given by the caller
 * Returns true if x4 is close enough along with the bary coords of point
 * projected onto the face, stored in info itself */
bool Collision::checkProximity(ImpulseInfo &info)
{
  /* Following "Robust Treatment of Collisions, Contact, and Friction
   * for Cloth Animation"'s styling */
  Vec3 x43 = *(info.x4) - *(info.x3);

  /* Point x4 should be within simulation->cloth_thickness distance from the
   * plane of the face */
  if (fabs(dot(x43, *(info.n))) > simulation->cloth_thickness) {
    return false;
  }

  Vec3 x13 = *(info.x1) - *(info.x3);
  Vec3 x23 = *(info.x2) - *(info.x3);

  EigenMat2 mat;
  mat << dot(x13, x13), dot(x13, x23), dot(x13, x23), dot(x23, x23);
  EigenVec2 vec;
  vec << dot(x13, x43), dot(x23, x43);
  EigenVec2 w = mat.colPivHouseholderQr().solve(vec);

  /* Characteristic length of triangle is square root of the area of
   * the triangle */
  double delta = simulation->cloth_thickness /
                 sqrt(0.5 * norm((normal(*(info.x1), *(info.x2), *(info.x3)))));
  info.bary_coords[0] = w[0];
  info.bary_coords[1] = w[1];
  info.bary_coords[2] = 1.0 - (w[0] + w[1]);

  /* barycentric coordinates must be within [-delta, 1 + delta] */
  for (int i = 0; i < 3; i++) {
    if (info.bary_coords[i] < -delta) {
      return false;
    }
    if (info.bary_coords[i] > 1 + delta) {
      return false;
    }
  }

  return true;
}

void Collision::checkProximityAndCalculateImpulse(ClothNode *cloth_node,
                                                  Face *face,
                                                  double coeff_friction)
{
  /* Currently, since the obstacle doesn't have a velocity term, we
   * need to consider this as Vec3(0) */
  Vec3 zero_vec = Vec3(0.0);
  ImpulseInfo info(&face->v[0]->node->x,
                   &face->v[1]->node->x,
                   &face->v[2]->node->x,
                   &cloth_node->x0,
                   &zero_vec,
                   &zero_vec,
                   &zero_vec,
                   &cloth_node->v,
                   &face->n,
                   &coeff_friction,
                   &cloth_node->mass,
                   Vec3(0.0));

  if (checkProximity(info)) {
    Vec3 impulse;
    if (calculateImpulse(info, impulse)) {
      cloth_node->impulse += impulse;
      cloth_node->impulse_count++;
    }
  }
}

void Collision::checkProximityAndCalculateImpulse(Node *node,
                                                  ClothFace *cloth_face,
                                                  double coeff_friction)
{
  /* Currently, since the obstacle doesn't have a velocity term, we
   * need to consider this as Vec3(0) */
  ClothNode *node_0 = static_cast<ClothNode *>(cloth_face->v[0]->node);
  ClothNode *node_1 = static_cast<ClothNode *>(cloth_face->v[1]->node);
  ClothNode *node_2 = static_cast<ClothNode *>(cloth_face->v[2]->node);
  Vec3 zero_vec = Vec3(0.0);
  ImpulseInfo info(&node_0->x0,
                   &node_1->x0,
                   &node_2->x0,
                   &node->x,
                   &node_0->v,
                   &node_1->v,
                   &node_2->v,
                   &zero_vec,
                   &cloth_face->n,
                   &coeff_friction,
                   &node_0->mass,
                   Vec3(0.0));

  if (checkProximity(info)) {
    Vec3 impulse;
    if (calculateImpulse(info, impulse)) {
      node_0->impulse += (info.bary_coords[0] * impulse);
      node_0->impulse_count++;
      node_1->impulse += (info.bary_coords[1] * impulse);
      node_1->impulse_count++;
      node_2->impulse += (info.bary_coords[2] * impulse);
      node_2->impulse_count++;
    }
  }
}

void Collision::checkProximityAndCalculateImpulse(ClothNode *cloth_node,
                                                  ClothFace *cloth_face,
                                                  double coeff_friction)
{
  ClothNode *node_0 = static_cast<ClothNode *>(cloth_face->v[0]->node);
  ClothNode *node_1 = static_cast<ClothNode *>(cloth_face->v[1]->node);
  ClothNode *node_2 = static_cast<ClothNode *>(cloth_face->v[2]->node);
  ImpulseInfo info(&node_0->x0,
                   &node_1->x0,
                   &node_2->x0,
                   &cloth_node->x0,
                   &node_0->v,
                   &node_1->v,
                   &node_2->v,
                   &cloth_node->v,
                   &cloth_face->n,
                   &coeff_friction,
                   &cloth_node->mass,
                   Vec3(0.0));

  if (checkProximity(info)) {
    Vec3 impulse;
    if (calculateImpulse(info, impulse)) {
      node_0->impulse += (info.bary_coords[0] * impulse);
      node_0->impulse_count++;
      node_1->impulse += (info.bary_coords[1] * impulse);
      node_1->impulse_count++;
      node_2->impulse += (info.bary_coords[2] * impulse);
      node_2->impulse_count++;

      cloth_node->impulse -= impulse;
      cloth_node->impulse_count++;
    }
  }
}

void Collision::checkProximityAndCalculateImpulse(ClothFace *cloth_face,
                                                  Face *obstacle_face,
                                                  double coeff_friction)
{
  for (int i = 0; i < 3; i++) {
    ClothNode *node = static_cast<ClothNode *>(cloth_face->v[i]->node);

    checkProximityAndCalculateImpulse(node, obstacle_face, coeff_friction);
  }
  for (int i = 0; i < 3; i++) {
    Node *node = obstacle_face->v[i]->node;

    checkProximityAndCalculateImpulse(node, cloth_face, coeff_friction);
  }
}

void Collision::checkProximityAndCalculateImpulse(ClothFace *cloth_face_0,
                                                  ClothFace *cloth_face_1,
                                                  double coeff_friction)
{
  for (int i = 0; i < 3; i++) {
    ClothNode *node = static_cast<ClothNode *>(cloth_face_0->v[i]->node);

    checkProximityAndCalculateImpulse(node, cloth_face_1, coeff_friction);
  }
  for (int i = 0; i < 3; i++) {
    ClothNode *node = static_cast<ClothNode *>(cloth_face_1->v[i]->node);

    checkProximityAndCalculateImpulse(node, cloth_face_0, coeff_friction);
  }
}

bool Collision::collisionTestVF(ClothNode *cloth_node, Face *face, Impact &r_impact)
{
  r_impact.nodes[0] = face->v[0]->node;
  r_impact.nodes[1] = face->v[1]->node;
  r_impact.nodes[2] = face->v[2]->node;
  r_impact.nodes[3] = static_cast<Node *>(cloth_node);
  /* TODO(ish): when obstacles can also move, need to switch them out
   * for x0 */
  Vec3 &x1 = face->v[0]->node->x;
  Vec3 &x2 = face->v[1]->node->x;
  Vec3 &x3 = face->v[2]->node->x;
  Vec3 &x4 = cloth_node->x0;
  Vec3 v1 = Vec3(0.0);
  Vec3 v2 = Vec3(0.0);
  Vec3 v3 = Vec3(0.0);
  Vec3 &v4 = cloth_node->v;
  Vec3 x21 = x2 - x1;
  Vec3 x31 = x3 - x1;
  Vec3 x41 = x4 - x1;
  Vec3 v21 = v2 - v1;
  Vec3 v31 = v3 - v1;
  Vec3 v41 = v4 - v1;

  double a = stp(v21, v31, v41);
  double b = stp(v21, x31, v41) + stp(v21, v31, x41) + stp(x21, v31, v41);
  double c = stp(v21, x31, x41) + stp(x21, v31, x41) + stp(x21, x31, v41);
  double d = stp(x21, x31, x41);

  array<double, 3> t;
  int num_sol = solveCubic(a, b, c, d, &t[0]);
  sort(t.begin(), t.begin() + num_sol);

  /* TODO(ish): once all the velocities are represented as actual_v *
   * timestep , t[i] will need to change to 1 instead of
   * collision_timestep */
  for (int i = 0; i < num_sol; i++) {
    if (t[i] < numeric_limits<double>::epsilon() ||
        t[i] > collision_timestep + numeric_limits<double>::epsilon()) {
      continue;
    }
    r_impact.time = t[i];
    /* Get the interpolated positions for the current time */
    Vec3 nx1 = x1 + (t[i] * v1);
    Vec3 nx2 = x2 + (t[i] * v2);
    Vec3 nx3 = x3 + (t[i] * v3);
    Vec3 nx4 = x4 + (t[i] * v4);
    r_impact.n = normalize(normal(nx1, nx2, nx3));
    Vec3 &bary_coords = r_impact.bary_coords;

    /* Need to check proximity of nx4 with the rest */
    Vec3 zero_vec = Vec3(0.0);
    double coeff_friction = 0.0d;
    Vec3 impulse;
    ImpulseInfo info(&nx1, &nx2, &nx3, &nx4, &r_impact.n);
    if (checkProximity(info)) {
      bary_coords = info.bary_coords;
      return true;
    }
  }
  return false;
}

void Collision::findImpacts(ClothFace *cloth_face, Face *obstacle_face, vector<Impact> &r_impacts)
{
  for (int i = 0; i < 3; i++) {
    ClothNode *node = static_cast<ClothNode *>(cloth_face->v[i]->node);
    Impact impact;

    if (collisionTestVF(node, obstacle_face, impact)) {
      r_impacts.push_back(impact);
    }
  }
}

static bool conflict(const Impact &i0, const Impact &i1)
{
  return is_in(i0.nodes[0], i1.nodes, 4) || is_in(i0.nodes[1], i1.nodes, 4) ||
         is_in(i0.nodes[2], i1.nodes, 4) || is_in(i0.nodes[3], i1.nodes, 4);
}

vector<Impact> Collision::findIndependentImpacts(vector<Impact> impacts)
{
  sort(impacts.begin(), impacts.end());
  vector<Impact> independent;
  int impacts_size = impacts.size();
  for (int i = 0; i < impacts_size; i++) {
    const Impact &impact = impacts[i];
    bool con = false;
    for (int j = 0; j < independent.size(); j++) {
      if (conflict(impact, independent[j])) {
        con = true;
      }
    }
    if (!con) {
      independent.push_back(impact);
    }
  }
  return independent;
}

ImpactZone *Collision::findOrCreateImpactZone(Node *node, vector<ImpactZone *> &r_zones)
{
  int r_zones_size = r_zones.size();
  for (int i = 0; i < r_zones_size; i++) {
    if (is_in(node, r_zones[i]->nodes)) {
      return r_zones[i];
    }
  }

  ImpactZone *zone = new ImpactZone;
  zone->nodes.push_back(node);
  r_zones.push_back(zone);
  return zone;
}

void ImpactZone::merge(ImpactZone *zone, vector<ImpactZone *> &r_zones)
{
  assert(zone);
  if (this == zone) {
    return;
  }
  append(nodes, zone->nodes);
  append(impacts, zone->impacts);
  exclude(zone, r_zones);
  delete zone;
}

void Collision::addToImpactZones(vector<Impact> &impacts, vector<ImpactZone *> &r_zones)
{
  int r_zones_size = r_zones.size();
  for (int i = 0; i < r_zones_size; i++) {
    r_zones[i]->active = false;
  }
  int impacts_size = impacts.size();
  for (int i = 0; i < impacts_size; i++) {
    const Impact &impact = impacts[i];
    Node *node = impact.nodes[3];
    ImpactZone *zone = findOrCreateImpactZone(node, r_zones);
    for (int j = 0; j < 4; j++) {
      zone->merge(findOrCreateImpactZone(impact.nodes[j], r_zones), r_zones);
    }
    zone->impacts.push_back(impact);
    zone->active = true;
  }
}

void Collision::rigidImpactZoneResolution(ImpactZone *zone)
{
  if (!zone->active) {
    return;
  }

  Vec3 xcm = Vec3(0.0);
  Vec3 vcm = Vec3(0.0);
  double total_mass = 0.0;

  /* TODO(ish): this part can be simplified after obstacles can also
   * move */
  double obstacle_mass = 0.1;
  int nodes_size = zone->nodes.size();
  /* cout << "nodes_size: " << nodes_size << endl; */
  int cloth_nodes_count = 0;
  int obstacle_nodes_count = 0;
  for (int i = 0; i < nodes_size; i++) {
    ClothNode *node;
    if (node = dynamic_cast<ClothNode *>(zone->nodes[i])) {
      xcm += node->mass * node->x0;
      vcm += node->mass * node->v;
      total_mass += node->mass;
      /* cout << "node->x0: " << node->x0 << " node->v: " << node->v << " node->mass: " <<
       * node->mass */
      /*      << endl; */
      cloth_nodes_count++;
    }
    else {
      xcm += obstacle_mass * zone->nodes[i]->x;
      vcm += Vec3(0.0);
      total_mass += obstacle_mass;
      /* cout << "zone->nodes[i]->x: " << zone->nodes[i]->x << " obstacle_mass: " << obstacle_mass
       */
      /*      << endl; */
      obstacle_nodes_count++;
    }
  }
  /* cout << "nodes_size: " << nodes_size << " cloth_nodes_count: " << cloth_nodes_count */
  /*      << " obstacle_nodes_count: " << obstacle_nodes_count << endl; */
  xcm /= total_mass;
  vcm /= total_mass;

  /* cout << "xcm: " << xcm << " vcm: " << vcm << " total_mass: " << total_mass << endl; */

  Vec3 L = Vec3(0.0);
  for (int i = 0; i < nodes_size; i++) {
    ClothNode *node;
    if (node = dynamic_cast<ClothNode *>(zone->nodes[i])) {
      L += node->mass * cross(node->x0 - xcm, node->v - vcm);
    }
    else {
      L += obstacle_mass * cross(zone->nodes[i]->x - xcm, Vec3(0.0) - vcm);
    }
  }
  /* cout << "L: " << L << endl; */
  /* if (norm2(L) < 1e-6) { */
  /*   return; */
  /* } */

  Mat3x3 identity = Mat3x3(1.0);
  Mat3x3 I;
  for (int i = 0; i < nodes_size; i++) {
    ClothNode *node;
    if (node = dynamic_cast<ClothNode *>(zone->nodes[i])) {
      Vec3 x0_xcm = node->x0 - xcm;
      I += node->mass * (norm2(x0_xcm) * identity - outer(x0_xcm, x0_xcm));
    }
    else {
      Vec3 x0_xcm = zone->nodes[i]->x - xcm;
      I += obstacle_mass * (norm2(x0_xcm) * identity - outer(x0_xcm, x0_xcm));
    }
  }
  /* cout << "I: " << I << endl; */

  Vec3 omega = inverse(I) * L;
  double omega_norm = norm(omega);
  Vec3 omega_normalized = omega / omega_norm;
  Vec3 omega_crossed = cross(omega_normalized, omega_normalized);
  /* cout << "omega: " << omega << " omega_norm: " << omega_norm << endl; */
  for (int i = 0; i < nodes_size; i++) {
    ClothNode *node;
    if (node = dynamic_cast<ClothNode *>(zone->nodes[i])) {
      /* node->v = vcm + cross(omega, node->x0 - xcm); */
      /* node->x = node->x0 + (collision_timestep * node->v); */
      Vec3 &x0 = node->x0;
      Vec3 x0_xcm = x0 - xcm;
      Vec3 xf = dot(x0_xcm, omega_normalized) * omega_normalized;
      Vec3 xr = x0_xcm - xf;

      node->x = xcm + (collision_timestep * vcm) + xf +
                (cos(collision_timestep * omega_norm) * xr) +
                cross(sin(collision_timestep * omega_norm) * omega_normalized, xr);
      node->v = (node->x - node->x0) / collision_timestep;
      /* cout << "node->x0: " << node->x0 << " node->x: " << node->x << " node->v: " << node->v */
      /*      << endl; */
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
  BVHTreeOverlap *overlap = NULL;
  overlap = BVHTree_overlap(cloth_mesh->bvh, obstacle_mesh->bvh, &overlap_size, NULL, NULL);
  if (!overlap) {
    return;
  }

  setImpulseToZero(cloth_mesh);
  /* TODO(ish): coeffient of friction should be a property of both the
   * cloth mesh as well as the obstacle_mesh, will need to implement
   * coefficient of friction for obstacle mesh */
  double coeff_friction = cloth_mesh->coeff_friction;

  for (int i = 0; i < overlap_size; i++) {
    int cloth_mesh_index = overlap[i].indexA;
    int obstacle_mesh_index = overlap[i].indexB;

    ClothFace *cloth_face = static_cast<ClothFace *>(cloth_mesh->faces[cloth_mesh_index]);
    Face *obstacle_face = obstacle_mesh->faces[obstacle_mesh_index];

    checkProximityAndCalculateImpulse(cloth_face, obstacle_face, coeff_friction);
  }
  /* Applying impulse to the velocity and calculating the new
   * position of the node */
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
    if (count > 0) {
      cloth_mesh->updateBVH();
    }
  }

  if (overlap) {
    delete[] overlap;
    overlap = NULL;
  }

  /* Finding impact zones for Rigid Impact Zone fail safe */
  int max_iter = 100;
  int iter;
  vector<ImpactZone *> zones;
  for (iter = 1; iter <= max_iter; iter++) {
    vector<Impact> impacts;
    overlap_size = 0;
    overlap = BVHTree_overlap(cloth_mesh->bvh, obstacle_mesh->bvh, &overlap_size, NULL, NULL);
    if (!overlap) {
      break;
    }
    for (int i = 0; i < overlap_size; i++) {
      int cloth_mesh_index = overlap[i].indexA;
      int obstacle_mesh_index = overlap[i].indexB;

      ClothFace *cloth_face = static_cast<ClothFace *>(cloth_mesh->faces[cloth_mesh_index]);
      Face *obstacle_face = obstacle_mesh->faces[obstacle_mesh_index];

      findImpacts(cloth_face, obstacle_face, impacts);
    }
    impacts = findIndependentImpacts(impacts);
    if (impacts.empty()) {
      /* This is a good thing, we have managed to resolve all the
       * collisions successfully */
      break;
    }
    addToImpactZones(impacts, zones);
    int zones_size = zones.size();
    for (int i = 0; i < zones_size; i++) {
      rigidImpactZoneResolution(zones[i]);
    }
    cloth_mesh->updateBVH();
    obstacle_mesh->updateBVH();

    if (overlap) {
      delete[] overlap;
      overlap = NULL;
    }
  }
  if (iter >= max_iter) {
    cout << "warning: collision failsafe iterations have been crossed!" << endl;
  }

  int zones_size = zones.size();
  for (int i = 0; i < zones_size; i++) {
    if (zones[i]) {
      delete zones[i];
      zones[i] = NULL;
    }
  }

  if (overlap) {
    delete[] overlap;
    overlap = NULL;
  }

  /* TODO(ish): Later on when the obstacle mesh is also affected by the
   * cloth/during self collision, the barycoords need to be carried
   * over to this part as well */
}

bool selfCollisionBVHCallback(void *userdata, int index_a, int index_b, int thread)
{
  /* No need to calculate for pairs twice */
  if (index_a < index_b) {
    ClothMesh *cloth_mesh = static_cast<ClothMesh *>(userdata);

    Face *face_0 = cloth_mesh->faces[index_a];
    Face *face_1 = cloth_mesh->faces[index_b];

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        if (face_0->v[i]->node == face_1->v[j]->node) {
          return false;
        }
      }
    }

    return true;
  }
  return false;
}

void Collision::solveSelfCollision(ClothMesh *cloth_mesh)
{
  /* Assuming that previously called functions have updated the bvh of
   * cloth_mesh */
  unsigned int overlap_size = 0;
  BVHTreeOverlap *overlap = NULL;
  overlap = BVHTree_overlap(cloth_mesh->bvh,
                            cloth_mesh->bvh,
                            &overlap_size,
                            selfCollisionBVHCallback,
                            static_cast<void *>(cloth_mesh));

  if (!overlap) {
    return;
  }

  setImpulseToZero(cloth_mesh);
  /* TODO(ish): coeffient of friction should be a property of both the
   * cloth mesh as well as the obstacle_mesh, will need to implement
   * coefficient of friction for obstacle mesh */
  double coeff_friction = cloth_mesh->coeff_friction;

  for (int i = 0; i < overlap_size; i++) {
    int cloth_index_0 = overlap[i].indexA;
    int cloth_index_1 = overlap[i].indexB;

    ClothFace *cloth_face_0 = static_cast<ClothFace *>(cloth_mesh->faces[cloth_index_0]);
    ClothFace *cloth_face_1 = static_cast<ClothFace *>(cloth_mesh->faces[cloth_index_1]);

    checkProximityAndCalculateImpulse(cloth_face_0, cloth_face_1, coeff_friction);
  }
  /* Applying impulse to the velocity and calculating the new
   * position of the node */
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
    if (count > 0) {
      cloth_mesh->updateBVH();
    }
  }

  /* TODO(ish): implement RIZ for self collisions */

  if (overlap) {
    delete[] overlap;
    overlap = NULL;
  }
}

void Collision::solveCollision(bool rebuild_bvh)
{
  if (rebuild_bvh) {
    deleteBVH();
    buildBVH();
  }

  /* TODO(ish): add collision steps */
  /* TODO(ish): self collisions */
  /* TODO(ish): add support for moving obstacle_meshes */
  ClothMesh *cloth_mesh = simulation->mesh;
  /* Need to update BVH, when cloth_mesh is modified, it is upto the
   * function that modified it to update the BVH at the right time */
  cloth_mesh->updateBVH();
  vector<Mesh *> &obstacle_meshes = simulation->obstacle_meshes;
  int obstacle_meshes_size = obstacle_meshes.size();
  for (int i = 0; i < obstacle_meshes_size; i++) {
    Mesh *obstacle_mesh = obstacle_meshes[i];
    /* Need to update BVH */
    obstacle_mesh->updateBVH();
    obstacle_mesh->updateFaceNormals();
    cloth_mesh->updateFaceNormals();

    solveCollision(cloth_mesh, obstacle_mesh);
  }

  solveSelfCollision(cloth_mesh);
}
