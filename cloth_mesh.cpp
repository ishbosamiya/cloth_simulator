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
