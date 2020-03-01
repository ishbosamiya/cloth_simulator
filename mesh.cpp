#include "mesh.hpp"

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

void Mesh::buildBVH()
{
  assert(bvh == NULL);
  bvh = new BVHNode((Primitive **)&faces[0], faces.size());
}

void Mesh::deleteBVH()
{
  delete bvh;
  bvh = NULL;
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
