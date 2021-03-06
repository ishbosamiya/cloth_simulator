#include "simulation.hpp"

void Simulation::calculateInertiaY()
{
  const int num_nodes = mesh->nodes.size();
  inertia_y.resize(num_nodes * 3, Eigen::NoChange);

  for (int i = 0; i < num_nodes; i++) {
    const ClothNode *node = static_cast<ClothNode *>(mesh->nodes[i]);
    Vec3 y = node->x + node->v * h;
    inertia_y.block_vector(i) = vec3ToEigen(y);
  }
}

void Simulation::calculateExternalForces()
{
  /* TODO(ish): only gravity and wind supported at the moment, add support for
   * more later */
  const int num_nodes = mesh->nodes.size();
  external_forces.resize(num_nodes * 3);
  external_forces.setZero();

  bool wind = false;
  for (int i = 0; i < num_nodes; i++) {
    external_forces[3 * i + 1] += -gravity_constant;
    if (wind) {
      external_forces.block_vector(i) += vec3ToEigen(wind_strength * wind_direction);
      external_forces.block_vector(i) += vec3ToEigen(randomZeroOne() * turbulence *
                                                     randomUnitVector());
    }
  }

  external_forces = mesh->mass_matrix * external_forces;
}

void Simulation::updatePosAndVelocity(const EigenVecX &new_pos)
{
  const int num_nodes = mesh->nodes.size();
  for (int i = 0; i < num_nodes; i++) {
    ClothNode *node = static_cast<ClothNode *>(mesh->nodes[i]);

    node->x0 = node->x;
    node->x = eigenToVec3(new_pos.block_vector(i));
    node->v = (node->x - node->x0) / h;
  }
}

void Simulation::evaluateJMatrix(EigenSparseMatrix &r_j)
{
  const int num_nodes = mesh->nodes.size();
  r_j.resize(num_nodes * 3, constraints.size() * 3);

  vector<EigenSparseMatrixTriplet> j_triplets;

  for (int i = 0; i < constraints.size(); i++) {
    constraints[i]->evaluateJMatrix(i, j_triplets);
  }

  r_j.setFromTriplets(j_triplets.begin(), j_triplets.end());
}

void Simulation::evaluateDVector(const EigenVecX &x, EigenVecX &r_d)
{
  r_d.resize(constraints.size() * 3);
  r_d.setZero();

  for (int i = 0; i < constraints.size(); i++) {
    constraints[i]->evaluateDVector(i, x, r_d);
  }
}

void Simulation::setWeightedLaplacianMatrix()
{
  const int num_node = mesh->nodes.size();
  weighted_laplacian.resize(num_node * 3, num_node * 3);

  std::vector<EigenSparseMatrixTriplet> l_triplets;
  for (int i = 0; i < constraints.size(); i++) {
    constraints[i]->evaluateWeightedLaplacian(l_triplets);
  }

  weighted_laplacian.setFromTriplets(l_triplets.begin(), l_triplets.end());
}

void Simulation::setJMatrix()
{
  evaluateJMatrix(j_matrix);
}

void Simulation::factorizeDirectSolverLLT(
    const EigenSparseMatrix &a,
    Eigen::SimplicialLLT<EigenSparseMatrix, Eigen::Upper> &r_llt_solver)
{
  EigenSparseMatrix a_prime = a;
  r_llt_solver.analyzePattern(a_prime);
  r_llt_solver.factorize(a_prime);

  double regularization = 0.00001;
  bool success = true;

  while (r_llt_solver.info() != Eigen::Success) {
    regularization *= 10;
    a_prime = a_prime + regularization * mesh->identity_matrix;
    r_llt_solver.factorize(a_prime);
    success = false;
  }

  if (!success) {
    cout << "warning: adding " << regularization << " identites.(llt solver)" << endl;
  }
}

void Simulation::prefactorize()
{
  /* static bool run_already = false; */
  /* if (run_already) { */
  /*   return; */
  /* } */
  /* run_already = true; */
  EigenSparseMatrix a;
  double h2 = h * h;

  setWeightedLaplacianMatrix();
  setJMatrix();
  a = mesh->mass_matrix + h2 * weighted_laplacian;

  factorizeDirectSolverLLT(a, prefactored_solver);
}

void Simulation::integrateLocalGlobalOneIteration(EigenVecX &r_x, bool run_prefactorization)
{
  if (run_prefactorization) {
    prefactorize();
  }

  EigenVecX d;
  evaluateDVector(r_x, d);

  EigenVecX b = mesh->mass_matrix * inertia_y + h * h * (j_matrix * d + external_forces);

  r_x = prefactored_solver.solve(b);
}

void Simulation::integrateOptimization()
{
  EigenVecX next_pos = inertia_y;

  for (int i = 0; i < iterations_per_frame; i++) {
    if (i == 0) {
      integrateLocalGlobalOneIteration(next_pos, true);
    }
    else {
      integrateLocalGlobalOneIteration(next_pos, false);
    }
  }

  updatePosAndVelocity(next_pos);
}

void Simulation::dampVelocity()
{
  EigenVec3 pos_mc(0.0d, 0.0d, 0.0d);
  EigenVec3 vel_mc(0.0d, 0.0d, 0.0d);
  double denominator = 0.0d;
  double mass = 0.0d;
  int num_nodes = mesh->nodes.size();

  for (int i = 0; i < num_nodes; i++) {
    const ClothNode *node = static_cast<ClothNode *>(mesh->nodes[i]);
    mass = mesh->mass_matrix.coeff(i * 3, i * 3);

    pos_mc += mass * vec3ToEigen(node->x);
    vel_mc += mass * vec3ToEigen(node->v);
    denominator += mass;
  }

  assert(denominator != 0.0d);
  pos_mc /= denominator;
  vel_mc /= denominator;

  EigenVec3 angular_momentum(0.0d, 0.0d, 0.0d);
  EigenVec3 r(0.0d, 0.0d, 0.0d);
  EigenMat3 inertia;
  EigenMat3 r_mat;
  inertia.setZero();
  r_mat.setZero();

  for (int i = 0; i < num_nodes; i++) {
    const ClothNode *node = static_cast<ClothNode *>(mesh->nodes[i]);
    mass = mesh->mass_matrix.coeff(i * 3, i * 3);

    r = vec3ToEigen(node->x) - pos_mc;
    angular_momentum += r.cross(mass * vec3ToEigen(node->v));

    r_mat.coeffRef(0, 1) = r[2];
    r_mat.coeffRef(0, 2) = -r[1];
    r_mat.coeffRef(1, 0) = -r[2];
    r_mat.coeffRef(1, 2) = r[0];
    r_mat.coeffRef(2, 0) = r[1];
    r_mat.coeffRef(2, 1) = -r[0];

    inertia += r_mat * r_mat.transpose() * mass;
  }

  EigenVec3 angular_vel = inertia.inverse() * angular_momentum;

  EigenVec3 delta_v(0.0d, 0.0d, 0.0d);
  for (int i = 0; i < num_nodes; ++i) {
    ClothNode *node = static_cast<ClothNode *>(mesh->nodes[i]);
    r = vec3ToEigen(node->x) - pos_mc;
    delta_v = vel_mc + angular_vel.cross(r) - vec3ToEigen(node->v);
    node->v += eigenToVec3(damping_coefficient * delta_v);
  }
}

static inline bool check_abcd(const Vec3 &p0,
                              const Vec3 &a0,
                              const Vec3 &b0,
                              const Vec3 &c0,
                              const Vec3 &p1,
                              const Vec3 &a1,
                              const Vec3 &b1,
                              const Vec3 &c1)
{
  Vec3 n0 = normal(a0, b0, c0);
  Vec3 n1 = normal(a1, b1, c1);
  Vec3 delta = normal(a1 - a0, b1 - b0, c1 - c0);
  Vec3 nX = (n0 + n1 - delta) * 0.5;

  Vec3 pa0 = p0 - a0;
  Vec3 pa1 = p1 - a1;

  float A = dot(n0, pa0);
  float B = dot(n1, pa1);
  float C = dot(nX, pa0);
  float D = dot(nX, pa1);
  float E = dot(n1, pa0);
  float F = dot(n0, pa1);

  if (A > 0 && B > 0 && (2 * C + F) > 0 && (2 * D + E) > 0) {
    return false;
  }

  if (A < 0 && B < 0 && (2 * C + F) < 0 && (2 * D + E) < 0) {
    return false;
  }

  return true;
}

static inline bool check_vf(ClothNode *node, ClothFace *face)
{
  const ClothNode *f_n0 = static_cast<ClothNode *>(face->v[0]->node);
  const ClothNode *f_n1 = static_cast<ClothNode *>(face->v[1]->node);
  const ClothNode *f_n2 = static_cast<ClothNode *>(face->v[2]->node);
  return check_abcd(node->x0, f_n0->x0, f_n1->x0, f_n2->x0, node->x, f_n0->x, f_n1->x, f_n2->x);
}

void Simulation::applyTransformations()
{
  mesh->applyTransformation();

  for (int i = 0; i < obstacle_meshes.size(); i++) {
    Mesh *ob_mesh = obstacle_meshes[i];
    ob_mesh->applyTransformation();
  }
}

void Simulation::unapplyTransformations()
{
  mesh->unapplyTransformation();

  for (int i = 0; i < obstacle_meshes.size(); i++) {
    Mesh *ob_mesh = obstacle_meshes[i];
    ob_mesh->unapplyTransformation();
  }
}

void Simulation::update()
{
  applyTransformations();
  calculateInertiaY();

  calculateExternalForces();

  integrateOptimization();

  /* TODO(ish): collision detection, adaptive remeshing */
  Collision collision(this);
  collision.solveCollision();

  dampVelocity();

  unapplyTransformations();
  mesh->shadeSmooth();
}

static void getAdjNode(const ClothEdge *edge, ClothNode **r_n1, ClothNode **r_n2)
{
  const ClothFace *f1 = static_cast<ClothFace *>(edge->adj_f[0]);
  const ClothFace *f2 = static_cast<ClothFace *>(edge->adj_f[1]);

  if (f1) {
    if (f1->v[0]->node != edge->n[0] && f1->v[0]->node != edge->n[1]) {
      *r_n1 = static_cast<ClothNode *>(f1->v[0]->node);
    }
    else if (f1->v[1]->node != edge->n[0] && f1->v[1]->node != edge->n[1]) {
      *r_n1 = static_cast<ClothNode *>(f1->v[1]->node);
    }
    else if (f1->v[2]->node != edge->n[0] && f1->v[2]->node != edge->n[1]) {
      *r_n1 = static_cast<ClothNode *>(f1->v[2]->node);
    }
  }

  if (f2) {
    if (f2->v[0]->node != edge->n[0] && f2->v[0]->node != edge->n[1]) {
      *r_n2 = static_cast<ClothNode *>(f2->v[0]->node);
    }
    else if (f2->v[1]->node != edge->n[0] && f2->v[1]->node != edge->n[1]) {
      *r_n2 = static_cast<ClothNode *>(f2->v[1]->node);
    }
    else if (f2->v[2]->node != edge->n[0] && f2->v[2]->node != edge->n[1]) {
      *r_n2 = static_cast<ClothNode *>(f2->v[2]->node);
    }
  }
}

void Simulation::setConstraints()
{
  clearConstraints();

  applyTransformations();

  /* TODO(ish): currently only supports simple springs, need to add
   * bending springs, etc. */
  const int num_edges = mesh->edges.size();
  for (int i = 0; i < num_edges; i++) {
    const ClothEdge *edge = static_cast<ClothEdge *>(mesh->edges[i]);

    double rest_length = norm(edge->n[0]->x - edge->n[1]->x);
    SpringConstraint *c = new SpringConstraint(
        &stiffness_stretch, edge->n[0]->index, edge->n[1]->index, rest_length);
    constraints.push_back(c);

    /* warning: only as a test for now */
    ClothNode *n1 = NULL, *n2 = NULL;
    getAdjNode(edge, &n1, &n2);
    if (n1 && n2) {
      rest_length = norm(n1->x - n2->x);
      c = new SpringConstraint(&stiffness_bending, n1->index, n2->index, rest_length);
      constraints.push_back(c);
    }
  }

  unapplyTransformations();
}

void Simulation::clearConstraints()
{
  for (int i = 0; i < constraints.size(); i++) {
    delete constraints[i];
  }
  constraints.clear();
}

void Simulation::reset()
{
  const int num_nodes = mesh->nodes.size();
  inertia_y.resize(num_nodes * 3);
  external_forces.resize(num_nodes * 3);

  /* TODO(ish): mass_matrix and identity_matrix initialization should
   * be done by ClothMesh */
  mesh->mass_matrix.resize(num_nodes * 3, num_nodes * 3);
  mesh->identity_matrix.resize(num_nodes * 3, num_nodes * 3);
  double mass = 1.0d;
  mass = mass / (double)num_nodes;
  for (int i = 0; i < num_nodes; i++) {
    ClothNode *node = static_cast<ClothNode *>(mesh->nodes[i]);
    node->mass = mass;
  }
  vector<EigenSparseMatrixTriplet> i_triplets;
  vector<EigenSparseMatrixTriplet> m_triplets;
  for (int i = 0; i < num_nodes * 3; i++) {
    i_triplets.push_back(EigenSparseMatrixTriplet(i, i, 1));
    m_triplets.push_back(EigenSparseMatrixTriplet(i, i, mass));
  }
  mesh->mass_matrix.setFromTriplets(m_triplets.begin(), m_triplets.end());
  mesh->identity_matrix.setFromTriplets(i_triplets.begin(), i_triplets.end());

  setConstraints();
}

bool Simulation::tryToTogglePinConstraint(const Vec3 &p0, const Vec3 &dir)
{
  applyTransformations();
  const int num_nodes = mesh->nodes.size();
  const int num_constraints = constraints.size();
  Vec3 p1;

  double ray_point_dist;
  double min_dist = 100.0d;
  int best_candidate = 0; /* TODO(ish): this might have to be -1 */

  /* Finding the nearest point */
  for (int i = 0; i < num_nodes; i++) {
    p1 = mesh->nodes[i]->x;

    ray_point_dist = norm(cross(p1 - p0, dir));
    if (ray_point_dist < min_dist) {
      min_dist = ray_point_dist;
      best_candidate = i;
    }
  }

  for (vector<Constraint *>::iterator i = constraints.begin(); i != constraints.end(); i++) {
    PinConstraint *pc;
    if (pc = dynamic_cast<PinConstraint *>(*i)) {
      ray_point_dist = norm(cross(pc->getPos() - p0, dir));
      if (ray_point_dist < min_dist) {
        min_dist = ray_point_dist;
        best_candidate = pc->getIndex();
      }
    }
  }

  if (min_dist > 0.1d) { /* TODO(ish): make the minimum distance to
                          * any node a user defined parameter */
    unapplyTransformations();
    return false;
  }

  bool current_state_on = false;
  for (vector<Constraint *>::iterator i = constraints.begin(); i != constraints.end(); i++) {
    PinConstraint *pc;
    if (pc = dynamic_cast<PinConstraint *>(*i)) {
      if (pc->getIndex() == best_candidate) {
        cout << "removed constraint for " << pc->getPos() << endl;
        current_state_on = true;
        constraints.erase(i);
        delete pc;
        break;
      }
    }
  }

  if (!current_state_on) {
    addPinConstraint(best_candidate);
    cout << "added constraint for " << mesh->nodes[best_candidate]->x << endl;
  }

  unapplyTransformations();
  return true;
}

void Simulation::addPinConstraint(int index)
{
  PinConstraint *pc = new PinConstraint(&stiffness_pin, index, mesh->nodes[index]->x);
  constraints.push_back(pc);
}

void Simulation::drawConstraints(glm::mat4 &projection,
                                 glm::mat4 &view,
                                 bool draw_stretch,
                                 bool draw_bending)
{
  const int num_constraints = constraints.size();
  vector<glm::vec3> pos_stretch;
  vector<unsigned int> indices_stretch;
  vector<glm::vec3> pos_bending;
  vector<unsigned int> indices_bending;
  int count_stretch = 0;
  int count_bending = 0;
  for (int i = 0; i < num_constraints; i++) {
    SpringConstraint *sc;
    if (sc = dynamic_cast<SpringConstraint *>(constraints[i])) {
      const ClothNode *node1 = static_cast<ClothNode *>(mesh->nodes[sc->getP1()]);
      const ClothNode *node2 = static_cast<ClothNode *>(mesh->nodes[sc->getP2()]);
      if (sc->getStiffnessPointer() == &stiffness_stretch && draw_stretch) {
        pos_stretch.push_back(glm::vec3(node1->x[0], node1->x[1], node1->x[2]));
        pos_stretch.push_back(glm::vec3(node2->x[0], node2->x[1], node2->x[2]));
        indices_stretch.push_back(count_stretch++);
        indices_stretch.push_back(count_stretch++);
      }
      else if (sc->getStiffnessPointer() == &stiffness_bending && draw_bending) {
        pos_bending.push_back(glm::vec3(node1->x[0], node1->x[1], node1->x[2]));
        pos_bending.push_back(glm::vec3(node2->x[0], node2->x[1], node2->x[2]));
        indices_bending.push_back(count_bending++);
        indices_bending.push_back(count_bending++);
      }
    }
    else {
      constraints[i]->draw(projection, view);
    }
  }

  static Shader line_shader("shaders/line.vert", "shaders/line.frag");
  line_shader.use();
  line_shader.setMat4("projection", projection);
  line_shader.setMat4("view", view);
  glm::mat4 model = glm::mat4(1.0f);
  model = glm::translate(model, vec3ToGlmVec3(mesh->pos));
  model = glm::scale(model, vec3ToGlmVec3(mesh->scale));
  line_shader.setMat4("model", model);
  line_shader.setVec4("color", 0.4, 0.8, 0.5, 1.0);

  if (draw_stretch) {
    GLLine line_stretch(pos_stretch, indices_stretch);
    line_stretch.draw();
  }
  if (draw_bending) {
    GLLine line_bending(pos_bending, indices_bending);
    line_shader.use();
    line_shader.setVec4("color", 0.8, 0.4, 0.7, 1.0);
    line_bending.draw();
  }
}
