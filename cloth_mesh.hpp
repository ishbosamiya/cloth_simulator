#ifndef CLOTH_MESH_HPP
#define CLOTH_MESH_HPP

#include <vector>
#include <cassert>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>

#include "math.hpp"
#include "misc.hpp"
#include "mesh.hpp"

using namespace std;

class ClothVert;
class ClothNode;
class ClothEdge;
class ClothFace;
class ClothMesh;

/* Important to note that ClothNode is the world space vertex which can
 * have UV space coordinates thus is split into ClothNode and
 * ClothVert.
 * Generally ClothNode would be called a vertex */

/* Stores the UV information and corresponding World Space Node */
class ClothVert : public Vert {
 public:
  Mat3x3 sizing; /* sizing information for the remeshing
                  * step */

  ClothVert() : Vert()
  {
  }
  ClothVert(const Vec3 &uv) : Vert(uv)
  {
  }
};

/* Stores the World Space coordinates */
class ClothNode : public Node {
 public:
  Vec3 x0;             /* previous world space position of node */
  Vec3 v;              /* world space velocity of node */
  double I;            /* impulse (used in collision) */
  int impulse_count;   /* impulse count for averaging the impulse */
  Vec3 impulse_normal; /* normal (may not be normalized) for averaging
                        * the impulse */

  ClothNode() : Node()
  {
  }
  ClothNode(const Vec3 &x, const Vec3 &v) : Node(x), x0(x), v(v)
  {
  }
  ClothNode(const Vec3 &x, const Vec3 &v, const Vec3 &n) : Node(x, n), x0(x), v(v)
  {
  }
};

/* Stores the ClothEdge data */
class ClothEdge : public Edge {
 public:
  ClothEdge() : Edge()
  {
  }

  ClothEdge(ClothNode *n0, ClothNode *n1) : Edge(n0, n1)
  {
  }
};

/* Stores the ClothFace data */
/* Only triangles */
class ClothFace : public Face {
 public:
  double area, mass;
  Mat3x3 dm, dm_inv; /* required data for remeshing step */

  ClothFace() : Face(), area(0.0f), mass(0.0f)
  {
    type = PRIMITIVE_FACE;
  }

  ClothFace(ClothVert *v0, ClothVert *v1, ClothVert *v2) : Face(v0, v1, v2)
  {
    type = PRIMITIVE_FACE;
  }

  bool boundingBox(AABB &r_box)
  {
    Vec3 min_v_x;
    Vec3 max_v_x;
    Vec3 min_v_x0;
    Vec3 max_v_x0;

    ClothNode *node1 = static_cast<ClothNode *>(v[0]->node);
    ClothNode *node2 = static_cast<ClothNode *>(v[1]->node);
    ClothNode *node3 = static_cast<ClothNode *>(v[2]->node);
    min_v_x[0] = min(node1->x[0], node2->x[0], node3->x[0]);
    min_v_x0[0] = min(node1->x0[0], node2->x0[0], node3->x0[0]);
    min_v_x[1] = min(node1->x[1], node2->x[1], node3->x[1]);
    min_v_x0[1] = min(node1->x0[1], node2->x0[1], node3->x0[1]);
    min_v_x[2] = min(node1->x[2], node2->x[2], node3->x[2]);
    min_v_x0[2] = min(node1->x0[2], node2->x0[2], node3->x0[2]);

    max_v_x[0] = max(node1->x[0], node2->x[0], node3->x[0]);
    max_v_x0[0] = max(node1->x0[0], node2->x0[0], node3->x0[0]);
    max_v_x[1] = max(node1->x[1], node2->x[1], node3->x[1]);
    max_v_x0[1] = max(node1->x0[1], node2->x0[1], node3->x0[1]);
    max_v_x[2] = max(node1->x[2], node2->x[2], node3->x[2]);
    max_v_x0[2] = max(node1->x0[2], node2->x0[2], node3->x0[2]);

    r_box = surroundingBox(AABB(min_v_x, max_v_x), AABB(min_v_x0, max_v_x0));

    return true;
  }
};

/* Stores the overall ClothMesh data */
class ClothMesh : public Mesh {
 private:
 public:
  ClothMesh() : Mesh()
  {
  }

  ClothMesh(const string &filename)
  {
    loadObj(filename);
  }

  ClothMesh(const string &filename, Shader *shader) : Mesh(shader)
  {
    loadObj(filename);
  }

  ClothMesh(const string &filename, Vec3 pos) : Mesh(pos)
  {
    loadObj(filename);
  }

  ClothMesh(const string &filename, Vec3 pos, Shader *shader) : Mesh(pos, shader)
  {
    loadObj(filename);
  }

  ClothMesh(const string &filename, Vec3 pos, Vec3 scale) : Mesh(pos, scale)
  {
    loadObj(filename);
  }

  ClothMesh(const string &filename, Vec3 pos, Vec3 scale, Shader *shader)
      : Mesh(pos, scale, shader)
  {
    loadObj(filename);
  }

  EigenSparseMatrix mass_matrix;     /* TODO(ish): initialize this */
  EigenSparseMatrix identity_matrix; /* TODO(ish): initialize this */

  void add(ClothVert *vert);
  void add(ClothNode *node);
  void add(ClothEdge *edge);
  void add(ClothFace *face);

  void remove(ClothVert *vert);
  void remove(ClothNode *node);
  void remove(ClothEdge *edge);
  void remove(ClothFace *face);

  void loadObj(const string &file);

  void applyTransformation();
  void unapplyTransformation();

  ~ClothMesh()
  {
  }
};

inline ClothEdge *getEdge(const ClothNode *n0, const ClothNode *n1)
{
  for (int i = 0; i < (int)n0->adj_e.size(); i++) {
    ClothEdge *edge = static_cast<ClothEdge *>(n0->adj_e[i]);
    if (edge->n[0] == n1 || edge->n[1] == n1) {
      return edge;
    }
  }
  return NULL;
}

#endif
