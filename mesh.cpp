#include "mesh.hpp"

/* Get Vert of edge whose node matches n[edge_node] */
Vert *Edge::getVert(int face_side, int edge_node)
{
  if (!adj_f[face_side]) {
    return NULL;
  }
  for (int i = 0; i < 3; i++) {
    if (adj_f[face_side]->v[i]->node == n[edge_node]) {
      return adj_f[face_side]->v[i];
    }
  }
  return NULL;
}

/* Get Vert of adj_f[face_side] that is not part of this edge */
Vert *Edge::getOtherVertOfFace(int face_side)
{
  if (!adj_f[face_side]) {
    return NULL;
  }
  for (int i = 0; i < 3; i++) {
    if (adj_f[face_side]->v[i]->node != n[0] && adj_f[face_side]->v[i]->node != n[1]) {
      return adj_f[face_side]->v[i];
    }
  }
  return NULL;
}

static void connectVertWithNode(Vert *vert, Node *node);

bool Edge::split(EditedElements &r_ee)
{
  if (this->adj_f[0] == NULL && this->adj_f[1] == NULL) {
    /* Edge with no face, no need to split it as of right now */
    return false;
  }
  /* Need to get the new Node */
  Node *n3 = new Node((this->n[0]->x + this->n[1]->x) * 0.5);
  r_ee.add(n3);

  /* Make the new Edge */
  Edge *e1 = new Edge(n[0], n3);
  Edge *e2 = new Edge(n3, n[1]);
  r_ee.add(e1);
  r_ee.add(e2);

  Vert *v3 = NULL;
  /* Iterate for both adjacent faces to remove the face, add the newly
   * formed edges, verts and faces */
  for (int i = 0; i < 2; i++) {
    Face *f = this->adj_f[i];
    if (f == NULL) {
      continue;
    }

    r_ee.remove(f);
    Vert *v0 = getVert(i, i);
    Vert *v1 = getVert(i, 1 - i);
    Vert *v2 = getOtherVertOfFace(i);
    Node *n0 = v0->node;
    Node *n1 = v1->node;
    Node *n2 = v2->node;

    /* Make the new Vert only if it wasn't made already by the
     * previous face consideration */
    if (v3 == NULL) {
      v3 = new Vert((v0->uv + v1->uv) * 0.5);
      r_ee.add(v3);
      connectVertWithNode(v3, n3);
    }

    /* Make the new Edge */
    Edge *e3 = new Edge(n2, n3);
    r_ee.add(e3);

    /* Make the new Face */
    Face *f0 = new Face(v0, v3, v2);
    Face *f1 = new Face(v3, v1, v2);
    r_ee.add(f0);
    r_ee.add(f1);
  }
  r_ee.remove(this);
  return true;
}

void Mesh::add(Vert *vert)
{
  verts.push_back(vert);
  vert->node = NULL;
  vert->adj_f.clear();
  vert->index = verts.size() - 1;
}

void Mesh::add(Node *node)
{
  nodes.push_back(node);
  node->adj_e.clear();
  for (int i = 0; i < node->verts.size(); i++) {
    node->verts[i]->node = node;
  }
  node->index = nodes.size() - 1;
}

void Mesh::add(Edge *edge)
{
  edges.push_back(edge);
  edge->adj_f[0] = NULL;
  edge->adj_f[1] = NULL;
  include(edge, edge->n[0]->adj_e);
  include(edge, edge->n[1]->adj_e);
  edge->index = edges.size() - 1;
}

static void add_edges_if_needed(Mesh &mesh, const Face *face)
{
  for (int i = 0; i < 3; i++) {
    Node *n0 = face->v[i]->node, *n1 = face->v[NEXT(i)]->node;
    if (getEdge(n0, n1) == NULL) {
      mesh.add(new Edge(n0, n1));
    }
  }
}

void Mesh::add(Face *face)
{
  faces.push_back(face);
  add_edges_if_needed(*this, face);
  for (int i = 0; i < 3; i++) {
    Vert *v0 = face->v[NEXT(i)];
    Vert *v1 = face->v[PREV(i)];
    include(face, v0->adj_f);
    Edge *e = getEdge(v0->node, v1->node);
    face->adj_e[i] = e;
    int side = e->n[0] == v0->node ? 0 : 1;
    e->adj_f[side] = face;
  }
  face->index = faces.size() - 1;
}

void Mesh::remove(Vert *vert)
{
  assert(vert->adj_f.empty()); /* ensure that adjacent faces don't
                                  exist */
  exclude(vert, verts);
}

void Mesh::remove(Node *node)
{
  assert(node->adj_e.empty()); /* ensure that adjacent edges don't
                                  exist */
  exclude(node, nodes);
}

void Mesh::remove(Edge *edge)
{
  assert(!edge->adj_f[0] && !edge->adj_f[1]); /* ensure that adjacent
                                                 faces don't exist */
  exclude(edge, edges);
  exclude(edge, edge->n[0]->adj_e);
  exclude(edge, edge->n[1]->adj_e);
}

void Mesh::remove(Face *face)
{
  exclude(face, faces);
  for (int i = 0; i < 3; i++) {
    Vert *v0 = face->v[NEXT(i)];
    exclude(face, v0->adj_f);
    Edge *e = face->adj_e[i];
    int side = e->n[0] == v0->node ? 0 : 1;
    e->adj_f[side] = NULL;
  }
}

void Mesh::setIndices()
{
  for (int i = 0; i < verts.size(); i++) {
    verts[i]->index = i;
  }
  for (int i = 0; i < faces.size(); i++) {
    faces[i]->index = i;
  }
  for (int i = 0; i < nodes.size(); i++) {
    nodes[i]->index = i;
  }
  for (int i = 0; i < edges.size(); i++) {
    edges[i]->index = i;
  }
}

static void getValidLine(istream &in, string &line)
{
  do {
    getline(in, line);
  } while (in && (line.length() == 0 || line[0] == '#'));
}

static void connectVertWithNode(Vert *vert, Node *node)
{
  vert->node = node;
  include(vert, node->verts);
}

static double angle(const Vec3 &x0, const Vec3 &x1, const Vec3 &x2)
{
  Vec3 e1 = normalize(x1 - x0);
  Vec3 e2 = normalize(x2 - x0);
  return acos(clamp(dot(e1, e2), -1., 1.));
}

static vector<Face *> triangulate(const vector<Vert *> &verts)
{
  int n = verts.size();
  double best_min_angle = 0;
  int best_root = -1;
  for (int i = 0; i < n; i++) {
    double min_angle = infinity;
    const Vert *vert0 = verts[i];
    for (int j = 2; j < n; j++) {
      const Vert *vert1 = verts[(i + j - 1) % n], *vert2 = verts[(i + j) % n];
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
  Vert *vert0 = verts[i];
  vector<Face *> tris;
  for (int j = 2; j < n; j++) {
    Vert *vert1 = verts[(i + j - 1) % n], *vert2 = verts[(i + j) % n];
    tris.push_back(new Face(vert0, vert1, vert2));
  }
  return tris;
}

void Mesh::loadObj(const string &file)
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
      Vec3 uv;
      linestream >> uv[0] >> uv[1];
      this->add(new Vert(uv));
    }
    else if (keyword == "v") {
      Vec3 x;
      linestream >> x[0] >> x[1] >> x[2];
      this->add(new Node(x, Vec3(0)));
    }
    else if (keyword == "vn") {
      Vec3 n;
      linestream >> n[0] >> n[1] >> n[2];
      normals.push_back(n);
    }
    else if (keyword == "e") {
      int n0, n1;
      linestream >> n0 >> n1;
      this->add(new Edge(this->nodes[n0 - 1], this->nodes[n1 - 1]));
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
          verts.push_back(new Vert(nodes.back()->x));
          this->add(verts.back());
        }
      }
      for (int v = 0; v < verts.size(); v++) {
        connectVertWithNode(verts[v], nodes[v]);
      }
      vector<Face *> faces = triangulate(verts);
      for (int f = 0; f < faces.size(); f++) {
        this->add(faces[f]);
      }
    }
  }
  normals.clear();
}

void Mesh::saveObj(const string &filename)
{
  /* TODO(ish): setIndices() here might be useless, some basic
   * testing, the indices remain the same between loading a mesh and
   * immediately doing this, might change when the mesh is modified */
  setIndices();
  fstream fout(filename.c_str(), ios::out);
  for (int i = 0; i < nodes.size(); i++) {
    const Node *node = nodes[i];
    fout << "v " << node->x[0] << " " << node->x[1] << " " << node->x[2] << endl;
  }
  for (int i = 0; i < verts.size(); i++) {
    const Vert *vert = verts[i];
    fout << "vt " << vert->uv[0] << " " << vert->uv[1] << endl;
  }
  for (int i = 0; i < nodes.size(); i++) {
    const Node *node = nodes[i];
    fout << "vn " << node->n[0] << " " << node->n[1] << " " << node->n[2] << endl;
  }
  for (int i = 0; i < faces.size(); i++) {
    const Face *face = faces[i];
    fout << "f " << face->v[0]->node->index + 1 << "/" << face->v[0]->index + 1 << "/"
         << face->v[0]->node->index + 1 << " " << face->v[1]->node->index + 1 << "/"
         << face->v[1]->index + 1 << "/" << face->v[0]->node->index + 1 << " "
         << face->v[2]->node->index + 1 << "/" << face->v[2]->index + 1 << "/"
         << face->v[0]->node->index + 1 << endl;
  }
}

GLMesh Mesh::convertToGLMesh()
{
  /* shadeSmooth(); */
  /* TODO(ish): this is just a test for now */
  setIndices();
  vector<GLVertex> gl_verts;
  vector<unsigned int> gl_indices;

  gl_verts.resize(nodes.size());
  for (int i = 0; i < nodes.size(); i++) {
    const Node *node = nodes[i];
    GLVertex gl_vert;
    gl_vert.x.x = node->x[0];
    gl_vert.x.y = node->x[1];
    gl_vert.x.z = node->x[2];

    gl_vert.uv.x = node->verts[0]->uv[0];
    gl_vert.uv.y = node->verts[0]->uv[1];

    gl_vert.n.x = node->n[0];
    gl_vert.n.y = node->n[1];
    gl_vert.n.z = node->n[2];

    gl_verts[i] = gl_vert;
  }

  for (int i = 0; i < faces.size(); i++) {
    const Face *face = faces[i];
    for (int j = 0; j < 3; j++) {
      gl_indices.push_back(face->v[j]->node->index);
    }
  }

  return GLMesh(gl_verts, gl_indices);
}

void Mesh::shadeSmooth()
{
  for (int i = 0; i < nodes.size(); i++) {
    Node *node = nodes[i];

    /* TODO(ish): There is a faster way to do this, right now there are
       duplicate faces for which the normals are calculated */
    Vec3 n(0.0f);
    for (int v = 0; v < node->verts.size(); v++) {
      const Vert *vert = node->verts[v];
      for (int f = 0; f < vert->adj_f.size(); f++) {
        const Face *face = vert->adj_f[f];

        int j = find(vert, face->v);
        int j1 = (j + 1) % 3;
        int j2 = (j + 2) % 3;

        Vec3 e1 = face->v[j1]->node->x - vert->node->x;
        Vec3 e2 = face->v[j2]->node->x - vert->node->x;

        n += cross(e1, e2) / (2 * norm2(e1) * norm2(e2));
      }
    }

    node->n = normalize(n);
  }
}

void Mesh::draw()
{
  setShaderModelMatrix();
  GLMesh gl_mesh = convertToGLMesh();
  gl_mesh.draw();
}

void Mesh::drawWireframe(glm::mat4 projection, glm::mat4 view, Vec4 color)
{
  int edge_len = edges.size();
  glEnable(GL_LINE_SMOOTH);
  glLineWidth(1.2);

  GPUVertFormat *format = immVertexFormat();
  uint pos = format->addAttribute("pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
  uint col = format->addAttribute("color", GPU_COMP_F32, 4, GPU_FETCH_FLOAT);

  static Shader smooth_shader("shaders/shader_3D_smooth_color.vert",
                              "shaders/shader_3D_smooth_color.frag");
  glm::mat4 model = glm::mat4(1.0);
  model = glm::translate(model, vec3ToGlmVec3(pos));
  model = glm::scale(model, vec3ToGlmVec3(scale));
  smooth_shader.use();
  smooth_shader.setMat4("projection", projection);
  smooth_shader.setMat4("view", view);
  smooth_shader.setMat4("model", model);

  immBegin(GPU_PRIM_LINES, edge_len * 2, &smooth_shader);

  for (int i = 0; i < edge_len; i++) {
    immAttr4f(col, color[0], color[1], color[2], color[3]);
    Vec3 &x1 = edges[i]->n[0]->x;
    immVertex3f(pos, x1[0], x1[1], x1[2]);

    immAttr4f(col, color[0], color[1], color[2], color[3]);
    Vec3 &x2 = edges[i]->n[1]->x;
    immVertex3f(pos, x2[0], x2[1], x2[2]);
  }

  immEnd();
}

void Mesh::applyTransformation()
{
  if (pos == Vec3(0, 0, 0) && scale == Vec3(1, 1, 1)) {
  }
  else {
    glm::mat4 model = glm::mat4(1.0);
    model = glm::translate(model, vec3ToGlmVec3(pos));
    model = glm::scale(model, vec3ToGlmVec3(scale));
    const int num_nodes = nodes.size();
    for (int i = 0; i < num_nodes; i++) {
      Node *node = nodes[i];
      node->x = glmVec4ToVec3(model * glm::vec4(vec3ToGlmVec3(node->x), 1.0));
    }
  }
}

void Mesh::unapplyTransformation()
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
      Node *node = nodes[i];
      node->x = glmVec4ToVec3(model_inv * glm::vec4(vec3ToGlmVec3(node->x), 1.0));
    }
  }
}

void Mesh::buildBVH(float epsilon)
{
  assert(bvh == NULL);
  int faces_size = faces.size();
  bvh = BVHTree_new(faces_size, epsilon, 4, 8);

  for (int i = 0; i < faces_size; i++) {
    float co[3][3];

    vec3ToFloatVec3(faces[i]->v[0]->node->x, co[0]);
    vec3ToFloatVec3(faces[i]->v[1]->node->x, co[1]);
    vec3ToFloatVec3(faces[i]->v[2]->node->x, co[2]);

    BVHTree_insert(bvh, i, co[0], 3);
  }

  BVHTree_balance(bvh);
}

void Mesh::updateBVH()
{
  assert(bvh != NULL);
  int faces_size = faces.size();

  for (int i = 0; i < faces_size; i++) {
    float co[3][3];

    vec3ToFloatVec3(faces[i]->v[0]->node->x, co[0]);
    vec3ToFloatVec3(faces[i]->v[1]->node->x, co[1]);
    vec3ToFloatVec3(faces[i]->v[2]->node->x, co[2]);

    BVHTree_update_node(bvh, i, co[0], NULL, 3);
  }

  BVHTree_update_tree(bvh);
}

void Mesh::deleteBVH()
{
  if (bvh) {
    BVHTree_free(bvh);
    bvh = NULL;
  }
}

void Mesh::updateFaceNormals()
{
  int faces_size = faces.size();

  for (int i = 0; i < faces_size; i++) {
    Vec3 &x0 = faces[i]->v[0]->node->x;
    Vec3 &x1 = faces[i]->v[1]->node->x;
    Vec3 &x2 = faces[i]->v[2]->node->x;

    faces[i]->n = normalize(normal(x0, x1, x2));
  }
}

void Mesh::deleteMesh()
{
  for (int i = 0; i < verts.size(); i++) {
    delete verts[i];
  }
  for (int i = 0; i < nodes.size(); i++) {
    delete nodes[i];
  }
  for (int i = 0; i < edges.size(); i++) {
    delete edges[i];
  }
  for (int i = 0; i < faces.size(); i++) {
    delete faces[i];
  }

  verts.clear();
  verts.shrink_to_fit();
  nodes.clear();
  nodes.shrink_to_fit();
  edges.clear();
  edges.shrink_to_fit();
  faces.clear();
  faces.shrink_to_fit();
}

void EditedElements::apply(Mesh &mesh)
{
  for (int i = 0; i < removed_faces.size(); i++) {
    mesh.remove(removed_faces[i]);
  }
  for (int i = 0; i < removed_edges.size(); i++) {
    mesh.remove(removed_edges[i]);
  }
  for (int i = 0; i < removed_nodes.size(); i++) {
    mesh.remove(removed_nodes[i]);
  }
  for (int i = 0; i < removed_verts.size(); i++) {
    mesh.remove(removed_verts[i]);
  }

  for (int i = 0; i < added_verts.size(); i++) {
    mesh.add(added_verts[i]);
  }
  for (int i = 0; i < added_nodes.size(); i++) {
    mesh.add(added_nodes[i]);
  }
  for (int i = 0; i < added_edges.size(); i++) {
    mesh.add(added_edges[i]);
  }
  for (int i = 0; i < added_faces.size(); i++) {
    mesh.add(added_faces[i]);
  }
}
