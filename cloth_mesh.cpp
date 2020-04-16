#include "cloth_mesh.hpp"

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
      Vec3 uv;
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
          verts.push_back(new ClothVert(nodes.back()->x));
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
