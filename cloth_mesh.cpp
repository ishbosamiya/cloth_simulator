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
  include(edge, edge->n[0]->adj_e);
  include(edge, edge->n[1]->adj_e);
  edge->index = edges.size() - 1;
}

static void add_edges_if_needed(ClothMesh &mesh, const ClothFace *face)
{
  for (int i = 0; i < 3; i++) {
    ClothNode *n0 = face->v[i]->node, *n1 = face->v[NEXT(i)]->node;
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
    ClothVert *v0 = face->v[NEXT(i)];
    ClothVert *v1 = face->v[PREV(i)];
    include(face, v0->adj_f);
    ClothEdge *e = getEdge(v0->node, v1->node);
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
  exclude(vert, verts);
}

void ClothMesh::remove(ClothNode *node)
{
  assert(node->adj_e.empty()); /* ensure that adjacent edges don't
                                  exist */
  exclude(node, nodes);
}

void ClothMesh::remove(ClothEdge *edge)
{
  assert(!edge->adj_f[0] && !edge->adj_f[1]); /* ensure that adjacent
                                                 faces don't exist */
  exclude(edge, edges);
  exclude(edge, edge->n[0]->adj_e);
  exclude(edge, edge->n[1]->adj_e);
}

void ClothMesh::remove(ClothFace *face)
{
  exclude(face, faces);
  for (int i = 0; i < 3; i++) {
    ClothVert *v0 = face->v[NEXT(i)];
    exclude(face, v0->adj_f);
    ClothEdge *e = face->adj_e[i];
    int side = e->n[0] == v0->node ? 0 : 1;
    e->adj_f[side] = NULL;
  }
}

void ClothMesh::setIndices()
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

static void connectVertWithNode(ClothVert *vert, ClothNode *node)
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

static vector<ClothFace *> triangulate(const vector<ClothVert *> &verts)
{
  int n = verts.size();
  double best_min_angle = 0;
  int best_root = -1;
  for (int i = 0; i < n; i++) {
    double min_angle = infinity;
    const ClothVert *vert0 = verts[i];
    for (int j = 2; j < n; j++) {
      const ClothVert *vert1 = verts[(i + j - 1) % n], *vert2 = verts[(i + j) % n];
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
  ClothVert *vert0 = verts[i];
  vector<ClothFace *> tris;
  for (int j = 2; j < n; j++) {
    ClothVert *vert1 = verts[(i + j - 1) % n], *vert2 = verts[(i + j) % n];
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
    else if (keyword == "e") {
      int n0, n1;
      linestream >> n0 >> n1;
      this->add(new ClothEdge(this->nodes[n0 - 1], this->nodes[n1 - 1]));
    }
    else if (keyword == "f") {
      vector<ClothVert *> verts;
      vector<ClothNode *> nodes;
      string w;
      while (linestream >> w) {
        stringstream wstream(w);
        int v, n;
        char c;
        wstream >> n >> c >> v;
        nodes.push_back(this->nodes[n - 1]);
        if (wstream) {
          verts.push_back(this->verts[v - 1]);
        }
        else if (!nodes.back()->verts.empty()) {
          verts.push_back(nodes.back()->verts[0]);
        }
        else {
          verts.push_back(new ClothVert(nodes.back()->x));
          this->add(verts.back());
        }
      }
      for (int v = 0; v < verts.size(); v++) {
        connectVertWithNode(verts[v], nodes[v]);
      }
      vector<ClothFace *> faces = triangulate(verts);
      for (int f = 0; f < faces.size(); f++) {
        this->add(faces[f]);
      }
    }
  }
}

void ClothMesh::deleteMesh()
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
