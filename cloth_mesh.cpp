#include "cloth_mesh.hpp"

double ClothVert::ClothAR_size(ClothVert *vert)
{
  Vec2 Uij = this->uv - vert->uv;
  return sqrt(norm2(Uij, this->sizing + vert->sizing) * 0.5);
}

double ClothEdge::ClothAR_size()
{
  ClothVert *v00 = static_cast<ClothVert *>(getVert(0, 0));
  ClothVert *v01 = static_cast<ClothVert *>(getVert(0, 1));
  ClothVert *v10 = static_cast<ClothVert *>(getVert(1, 0));
  ClothVert *v11 = static_cast<ClothVert *>(getVert(1, 1));
  double size = 0.0;
  if (this->adj_f[0]) {
    size += v00->ClothAR_size(v01);
  }
  if (this->adj_f[1]) {
    size += v10->ClothAR_size(v11);
  }

  return (this->adj_f[0] && this->adj_f[1]) ? size * 0.5 : size;
}

/* Get ClothVert of edge whose node matches n[edge_node] */
ClothVert *ClothEdge::getVert(int face_side, int edge_node)
{
  if (!adj_f[face_side]) {
    return NULL;
  }
  for (int i = 0; i < 3; i++) {
    if (static_cast<ClothNode *>(adj_f[face_side]->v[i]->node) ==
        static_cast<ClothNode *>(n[edge_node])) {
      return static_cast<ClothVert *>(adj_f[face_side]->v[i]);
    }
  }
  return NULL;
}

/* Get Vert of adj_f[face_side] that is not part of this edge */
ClothVert *ClothEdge::getOtherVertOfFace(int face_side)
{
  if (!adj_f[face_side]) {
    return NULL;
  }
  for (int i = 0; i < 3; i++) {
    if (static_cast<ClothNode *>(adj_f[face_side]->v[i]->node) != static_cast<ClothNode *>(n[0]) &&
        static_cast<ClothNode *>(adj_f[face_side]->v[i]->node) != static_cast<ClothNode *>(n[1])) {
      return static_cast<ClothVert *>(adj_f[face_side]->v[i]);
    }
  }
  return NULL;
}

ClothVert *ClothEdge::getOppositeVert(int face_side)
{
  if (!adj_f[face_side]) {
    return NULL;
  }

  ClothNode *node = static_cast<ClothNode *>(n[face_side]);
  for (int i = 0; i < 3; i++) {
    if (static_cast<ClothNode *>(adj_f[face_side]->v[i]->node) == node) {
      return static_cast<ClothVert *>(adj_f[face_side]->v[PREV(i)]);
    }
  }

  return NULL;
}

static void connectVertWithNode(ClothVert *vert, ClothNode *node);

bool ClothEdge::split(EditedElements &r_ee)
{
  if (this->adj_f[0] == NULL && this->adj_f[1] == NULL) {
    /* ClothEdge with no face, no need to split it as of right now */
    return false;
  }
  /* Need to get the new ClothNode */
  ClothNode *n3 = new ClothNode((this->n[0]->x + this->n[1]->x) * 0.5);
  r_ee.add(n3);

  /* Make the new ClothEdge */
  ClothEdge *e1 = new ClothEdge(n[0], n3);
  ClothEdge *e2 = new ClothEdge(n3, n[1]);
  r_ee.add(e1);
  r_ee.add(e2);

  ClothVert *v3s[2] = {NULL, NULL};
  /* Iterate for both adjacent faces to remove the face, add the newly
   * formed edges, verts and faces */
  for (int i = 0; i < 2; i++) {
    ClothFace *f = static_cast<ClothFace *>(this->adj_f[i]);
    if (f == NULL) {
      continue;
    }

    r_ee.remove(f);
    ClothVert *v0 = getVert(i, i);
    ClothVert *v1 = getVert(i, 1 - i);
    ClothVert *v2 = getOppositeVert(i);
    ClothNode *n0 = static_cast<ClothNode *>(v0->node);
    ClothNode *n1 = static_cast<ClothNode *>(v1->node);
    ClothNode *n2 = static_cast<ClothNode *>(v2->node);

    /* Make the new ClothVert only if it wasn't made already by the
     * previous face consideration */
    if (i == 0 || this->isOnSeamOrBoundary()) {
      v3s[i] = new ClothVert((v0->uv + v1->uv) * 0.5);
      connectVertWithNode(v3s[i], n3);
      r_ee.add(v3s[i]);
    }
    else {
      v3s[i] = v3s[0];
    }

    /* Make the new ClothEdge */
    ClothEdge *e3 = new ClothEdge(n2, n3);
    r_ee.add(e3);

    /* Make the new ClothFace */
    ClothFace *f0 = new ClothFace(v0, v3s[i], v2);
    ClothFace *f1 = new ClothFace(v3s[i], v1, v2);
    r_ee.add(f0);
    r_ee.add(f1);
  }
  r_ee.remove(this);
  return true;
}

bool ClothEdge::collapse(int remove_index, EditedElements &r_ee)
{
  assert(remove_index == 0 || remove_index == 1);

  ClothNode *n0 = static_cast<ClothNode *>(this->n[remove_index]);
  ClothNode *n1 = static_cast<ClothNode *>(this->n[1 - remove_index]);
  /* Remove n0 */
  r_ee.remove(n0);

  /* Remove all the adjacent edges to the node to be removed, in this
   * case n0 */
  int adj_e_size = n0->adj_e.size();
  for (int i = 0; i < adj_e_size; i++) {
    ClothEdge *e_adj = static_cast<ClothEdge *>(n0->adj_e[i]);
    /* remove the adjacent edge */
    r_ee.remove(e_adj);

    /* Make a new edge only if the new edge doesn't already exist and
     * it won't be an edge between n1 & n1 itself */
    ClothNode *n_other = static_cast<ClothNode *>(e_adj->n[0]) == n0 ?
                             static_cast<ClothNode *>(e_adj->n[1]) :
                             static_cast<ClothNode *>(e_adj->n[0]);
    if (n_other != n1 && !getEdge(n_other, n1)) {
      ClothEdge *e_new = new ClothEdge(n1, n_other);
      r_ee.add(e_new);
    }
  }

  for (int i = 0; i < 2; i++) {
    /* Get the verts with similar naming as the nodes */
    ClothVert *v0 = getVert(i, remove_index);
    ClothVert *v1 = getVert(i, 1 - remove_index);

    /* v0 should exist and it shouldn't have be the same as the
     * previously removed v0 */
    if (!v0 || (i == 1 && v0 == getVert(0, remove_index))) {
      continue;
    }
    /* Remove the ClothVert v0 */
    r_ee.remove(v0);
    /* Remove all adjacent faces to v0 */
    int adj_f_size = v0->adj_f.size();
    for (int j = 0; j < adj_f_size; j++) {
      ClothFace *f = static_cast<ClothFace *>(v0->adj_f[j]);
      r_ee.remove(f);
      ClothVert *vs[3] = {static_cast<ClothVert *>(f->v[0]),
                          static_cast<ClothVert *>(f->v[1]),
                          static_cast<ClothVert *>(f->v[2])};
      if (!is_in(v1, vs)) {
        /* The verts of the new ClothFace made will be similar to the old
         * face except when v1 was already part of the previous face */
        replace(v0, v1, vs);
        ClothFace *f_new = new ClothFace(vs[0], vs[1], vs[2]);
        r_ee.add(f_new);
      }
    }
  }
  return true;
}

bool ClothEdge::flip(EditedElements &r_ee)
{
  ClothVert *v0 = getVert(0, 0);
  if (v0 == NULL) {
    return false;
  }
  ClothVert *v1 = getVert(1, 1);
  if (v1 == NULL) {
    return false;
  }
  ClothVert *v2 = getOppositeVert(0);
  if (v2 == NULL) {
    return false;
  }
  ClothVert *v3 = getOppositeVert(1);
  if (v3 == NULL) {
    return false;
  }

  /* Remove this ClothEdge */
  r_ee.remove(this);

  /* Remove adjacent ClothFaces, we no longer need to check if they exist
   * or not because getVert() and getOtherVertOfFace() also needs
   * there to be adjacent ClothFaces */
  ClothFace *f0 = static_cast<ClothFace *>(this->adj_f[0]);
  r_ee.remove(f0);
  ClothFace *f1 = static_cast<ClothFace *>(this->adj_f[1]);
  r_ee.remove(f1);

  /* Create new ClothFaces with the correct ordering of ClothVerts */
  ClothFace *f2 = new ClothFace(v0, v3, v2);
  ClothFace *f3 = new ClothFace(v1, v2, v3);
  r_ee.add(f2);
  r_ee.add(f3);

  /* Create new ClothEdge between v3 and v2 */
  ClothEdge *edge = new ClothEdge(v3->node, v2->node);
  r_ee.add(edge);

  return true;
}

ClothVert *ClothNode::adjacent(ClothVert *other)
{
  ClothEdge *edge = getEdge(this, static_cast<ClothNode *>(other->node));
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      if (static_cast<ClothVert *>(edge->getVert(j, i)) == other) {
        return static_cast<ClothVert *>(edge->getVert(j, 1 - i));
      }
    }
  }

  return NULL;
}

void ClothMesh::add(ClothVert *vert)
{
  verts.push_back(vert);
  vert->node = NULL;
  vert->adj_f.clear();
  vert->index = verts.size() - 1;
}

void ClothMesh::add(ClothNode *node)
{
  nodes.push_back(node);
  node->adj_e.clear();
  for (int i = 0; i < node->verts.size(); i++) {
    node->verts[i]->node = node;
  }
  node->index = nodes.size() - 1;
}

void ClothMesh::add(ClothEdge *edge)
{
  edges.push_back(edge);
  edge->adj_f[0] = NULL;
  edge->adj_f[1] = NULL;
  include(static_cast<Edge *>(edge), edge->n[0]->adj_e);
  include(static_cast<Edge *>(edge), edge->n[1]->adj_e);
  edge->index = edges.size() - 1;
}

static void add_edges_if_needed(ClothMesh &mesh, const ClothFace *face)
{
  for (int i = 0; i < 3; i++) {
    ClothNode *n0 = static_cast<ClothNode *>(face->v[i]->node),
              *n1 = static_cast<ClothNode *>(face->v[NEXT(i)]->node);
    if (getEdge(n0, n1) == NULL) {
      mesh.add(new ClothEdge(n0, n1));
    }
  }
}

void ClothMesh::add(ClothFace *face)
{
  faces.push_back(face);
  add_edges_if_needed(*this, face);
  for (int i = 0; i < 3; i++) {
    ClothVert *v0 = static_cast<ClothVert *>(face->v[NEXT(i)]);
    ClothVert *v1 = static_cast<ClothVert *>(face->v[PREV(i)]);
    include(static_cast<Face *>(face), v0->adj_f);
    ClothEdge *e = static_cast<ClothEdge *>(getEdge(v0->node, v1->node));
    face->adj_e[i] = e;
    int side = e->n[0] == v0->node ? 0 : 1;
    e->adj_f[side] = face;
  }
  face->index = faces.size() - 1;
}

void ClothMesh::remove(ClothVert *vert)
{
  assert(vert->adj_f.empty()); /* ensure that adjacent faces don't
                                  exist */
  exclude(static_cast<Vert *>(vert), verts);
}

void ClothMesh::remove(ClothNode *node)
{
  assert(node->adj_e.empty()); /* ensure that adjacent edges don't
                                  exist */
  exclude(static_cast<Node *>(node), nodes);
}

void ClothMesh::remove(ClothEdge *edge)
{
  assert(!edge->adj_f[0] && !edge->adj_f[1]); /* ensure that adjacent
                                                 faces don't exist */
  exclude(static_cast<Edge *>(edge), edges);
  exclude(static_cast<Edge *>(edge), edge->n[0]->adj_e);
  exclude(static_cast<Edge *>(edge), edge->n[1]->adj_e);
}

void ClothMesh::remove(ClothFace *face)
{
  exclude(static_cast<Face *>(face), faces);
  for (int i = 0; i < 3; i++) {
    ClothVert *v0 = static_cast<ClothVert *>(face->v[NEXT(i)]);
    exclude(static_cast<Face *>(face), v0->adj_f);
    ClothEdge *e = static_cast<ClothEdge *>(face->adj_e[i]);
    int side = e->n[0] == v0->node ? 0 : 1;
    e->adj_f[side] = NULL;
  }
}

bool ClothMesh::exists(const ClothEdge *edge)
{
  int edges_size = edges.size();
  for (int i = 0; i < edges_size; i++) {
    if (static_cast<ClothEdge *>(edges[i]) == edge) {
      return true;
    }
  }
  return false;
}

static void getValidLine(istream &in, string &line)
{
  do {
    getline(in, line);
  } while (in && (line.length() == 0 || line[0] == '#'));
}

static void connectVertWithNode(ClothVert *vert, ClothNode *node)
{
  vert->node = node;
  include(static_cast<Vert *>(vert), node->verts);
}

static double angle(const Vec3 &x0, const Vec3 &x1, const Vec3 &x2)
{
  Vec3 e1 = normalize(x1 - x0);
  Vec3 e2 = normalize(x2 - x0);
  return acos(clamp(dot(e1, e2), -1., 1.));
}

static vector<ClothFace *> triangulate(const vector<Vert *> &verts)
{
  int n = verts.size();
  double best_min_angle = 0;
  int best_root = -1;
  for (int i = 0; i < n; i++) {
    double min_angle = infinity;
    const ClothVert *vert0 = static_cast<ClothVert *>(verts[i]);
    for (int j = 2; j < n; j++) {
      const ClothVert *vert1 = static_cast<ClothVert *>(verts[(i + j - 1) % n]),
                      *vert2 = static_cast<ClothVert *>(verts[(i + j) % n]);
      min_angle = min(min_angle,
                      angle(vert0->node->x, vert1->node->x, vert2->node->x),
                      angle(vert1->node->x, vert2->node->x, vert0->node->x),
                      angle(vert2->node->x, vert0->node->x, vert1->node->x));
    }
    if (min_angle > best_min_angle) {
      best_min_angle = min_angle;
      best_root = i;
    }
  }
  int i = best_root;
  ClothVert *vert0 = static_cast<ClothVert *>(verts[i]);
  vector<ClothFace *> tris;
  for (int j = 2; j < n; j++) {
    ClothVert *vert1 = static_cast<ClothVert *>(verts[(i + j - 1) % n]),
              *vert2 = static_cast<ClothVert *>(verts[(i + j) % n]);
    tris.push_back(new ClothFace(vert0, vert1, vert2));
  }
  return tris;
}

void ClothMesh::loadObj(const string &file)
{
  /* TODO(Ish): need to delete the existing mesh structure before
   * loading obj */
  fstream fin(file.c_str(), ios::in);
  if (!fin) {
    cout << "error: No file found at " << file << endl;
    return;
  }

  vector<Vec3> normals;
  while (fin) {
    string line;
    getValidLine(fin, line);
    stringstream linestream(line);
    string keyword;
    linestream >> keyword;
    if (keyword == "vt") {
      Vec2 uv;
      linestream >> uv[0] >> uv[1];
      this->add(new ClothVert(uv));
    }
    else if (keyword == "v") {
      Vec3 x;
      linestream >> x[0] >> x[1] >> x[2];
      this->add(new ClothNode(x, Vec3(0)));
    }
    else if (keyword == "vn") {
      Vec3 n;
      linestream >> n[0] >> n[1] >> n[2];
      normals.push_back(n);
    }
    else if (keyword == "e") {
      int n0, n1;
      linestream >> n0 >> n1;
      this->add(new ClothEdge(static_cast<ClothNode *>(this->nodes[n0 - 1]),
                              static_cast<ClothNode *>(this->nodes[n1 - 1])));
    }
    else if (keyword == "f") {
      vector<Vert *> verts;
      vector<Node *> nodes;
      string w;
      while (linestream >> w) {
        stringstream wstream(w);
        int v, n, vn; /* vt, v, vn */
        char c, c2;
        wstream >> n >> c >> v >> c2 >> vn;
        nodes.push_back(this->nodes[n - 1]);
        this->nodes[n - 1]->n = normals[vn - 1];
        if (wstream) {
          verts.push_back(this->verts[v - 1]);
        }
        else if (!nodes.back()->verts.empty()) {
          verts.push_back(nodes.back()->verts[0]);
        }
        else {
          verts.push_back(new ClothVert(Vec2(nodes.back()->x[0], nodes.back()->x[1])));
          this->add(static_cast<ClothVert *>(verts.back()));
        }
      }
      for (int v = 0; v < verts.size(); v++) {
        connectVertWithNode(static_cast<ClothVert *>(verts[v]),
                            static_cast<ClothNode *>(nodes[v]));
      }
      vector<ClothFace *> faces = triangulate(verts);
      for (int f = 0; f < faces.size(); f++) {
        this->add(faces[f]);
      }
    }
  }
  normals.clear();
}

void ClothMesh::applyTransformation()
{
  if (pos == Vec3(0, 0, 0) && scale == Vec3(1, 1, 1)) {
  }
  else {
    glm::mat4 model = glm::mat4(1.0);
    model = glm::translate(model, vec3ToGlmVec3(pos));
    model = glm::scale(model, vec3ToGlmVec3(scale));
    const int num_nodes = nodes.size();
    for (int i = 0; i < num_nodes; i++) {
      ClothNode *node = static_cast<ClothNode *>(nodes[i]);
      node->x = glmVec4ToVec3(model * glm::vec4(vec3ToGlmVec3(node->x), 1.0));
      node->x0 = glmVec4ToVec3(model * glm::vec4(vec3ToGlmVec3(node->x0), 1.0));
    }
  }
}

void ClothMesh::unapplyTransformation()
{
  if (pos == Vec3(0, 0, 0) && scale == Vec3(1, 1, 1)) {
  }
  else {
    glm::mat4 model_inv = glm::mat4(1.0);
    model_inv = glm::translate(model_inv, vec3ToGlmVec3(pos));
    model_inv = glm::scale(model_inv, vec3ToGlmVec3(scale));
    model_inv = glm::inverse(model_inv);
    const int num_nodes = nodes.size();
    for (int i = 0; i < num_nodes; i++) {
      ClothNode *node = static_cast<ClothNode *>(nodes[i]);
      node->x = glmVec4ToVec3(model_inv * glm::vec4(vec3ToGlmVec3(node->x), 1.0));
      node->x0 = glmVec4ToVec3(model_inv * glm::vec4(vec3ToGlmVec3(node->x0), 1.0));
    }
  }
}

void ClothMesh::updateBVH()
{
  assert(bvh != NULL);
  int faces_size = faces.size();

  for (int i = 0; i < faces_size; i++) {
    float co[3][3];
    float co_moving[3][3];
    ClothNode *node_0 = static_cast<ClothNode *>(faces[i]->v[0]->node);
    ClothNode *node_1 = static_cast<ClothNode *>(faces[i]->v[1]->node);
    ClothNode *node_2 = static_cast<ClothNode *>(faces[i]->v[2]->node);

    vec3ToFloatVec3(node_0->x0, co[0]);
    vec3ToFloatVec3(node_1->x0, co[1]);
    vec3ToFloatVec3(node_2->x0, co[2]);

    vec3ToFloatVec3(node_0->x, co_moving[0]);
    vec3ToFloatVec3(node_1->x, co_moving[1]);
    vec3ToFloatVec3(node_2->x, co_moving[2]);

    BVHTree_update_node(bvh, i, co[0], co_moving[0], 3);
  }

  BVHTree_update_tree(bvh);
}

void ClothMesh::updateFaceNormals()
{
  int faces_size = faces.size();

  for (int i = 0; i < faces_size; i++) {
    Vec3 &x0 = static_cast<ClothNode *>(faces[i]->v[0]->node)->x0;
    Vec3 &x1 = static_cast<ClothNode *>(faces[i]->v[1]->node)->x0;
    Vec3 &x2 = static_cast<ClothNode *>(faces[i]->v[2]->node)->x0;

    faces[i]->n = normalize(normal(x0, x1, x2));
  }
}

void ClothMesh::drawVelocity(glm::mat4 &projection, glm::mat4 &view)
{
  static Shader smooth_shader("shaders/shader_3D_smooth_color.vert",
                              "shaders/shader_3D_smooth_color.frag");
  glm::mat4 model = glm::mat4(1.0);
  smooth_shader.use();
  smooth_shader.setMat4("projection", projection);
  smooth_shader.setMat4("view", view);
  smooth_shader.setMat4("model", model);

  glEnable(GL_LINE_SMOOTH);
  glLineWidth(2);

  int num_nodes = nodes.size();

  GPUVertFormat *format = immVertexFormat();
  uint pos = format->addAttribute("pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
  uint col = format->addAttribute("color", GPU_COMP_F32, 4, GPU_FETCH_FLOAT);

  immBegin(GPU_PRIM_LINES, num_nodes * 2, &smooth_shader);

  for (int i = 0; i < num_nodes; i++) {
    ClothNode *node = static_cast<ClothNode *>(nodes[i]);

    const Vec3 &start_pos = node->x;
    const double speed = norm(node->v);
    const double length_of_line = 0.04;
    const Vec3 end_pos = start_pos + (length_of_line * node->v / speed);
    const Vec3 color0 = Vec3(0.0, 1.0, 0.0);
    const Vec3 color1 = Vec3(1.0, 0.0, 0.0);
    const double max_speed = 3.0;
    const double color_weight = clamp(speed / max_speed, 0.0, 1.0);
    const Vec3 final_color = mix(color0, color1, color_weight);

    immAttr4f(col, 0.0, 0.0, 0.0, 1.0);
    immVertex3f(pos, start_pos[0], start_pos[1], start_pos[2]);

    immAttr4f(col, final_color[0], final_color[1], final_color[2], 1.0);
    immVertex3f(pos, end_pos[0], end_pos[1], end_pos[2]);
  }

  immEnd();
}
