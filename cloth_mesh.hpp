#ifndef CLOTH_MESH_HPP
#define CLOTH_MESH_HPP

#include <vector>
#include <cassert>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <glad/glad.h>

#include "math.hpp"
#include "misc.hpp"
#include "mesh.hpp"
#include "gpu_immediate.hpp"

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
  Mat2x2 sizing; /* sizing information for the remeshing
                  * step */

  ClothVert() : Vert()
  {
  }
  ClothVert(const Vec2 &uv) : Vert(uv)
  {
  }

  double ClothAR_size(ClothVert *vert);
};

/* Stores the World Space coordinates */
class ClothNode : public Node {
 public:
  Vec3 x0;           /* previous world space position of node */
  Vec3 v;            /* world space velocity of node */
  int impulse_count; /* impulse count for averaging the impulse */
  Vec3 impulse;      /* impulse with the direction for averaging
                      * the impulse */
  double mass;       /* Mass of Node */

  ClothNode() : Node()
  {
  }
  ClothNode(const Vec3 &x) : Node(x)
  {
  }
  ClothNode(const Vec3 &x, const Vec3 &v) : Node(x), x0(x), v(v)
  {
  }
  ClothNode(const Vec3 &x, const Vec3 &v, const Vec3 &n) : Node(x, n), x0(x), v(v)
  {
  }

  /* Get ClothVert on the opposite side of this ClothNode for the ClothEdge created by this
   * ClothNode and other->node */
  ClothVert *adjacent(ClothVert *other);
};

/* Stores the ClothEdge data */
class ClothEdge : public Edge {
 public:
  ClothEdge() : Edge()
  {
  }
  ClothEdge(Node *n0, Node *n1) : Edge(n0, n1)
  {
  }
  ClothEdge(ClothNode *n0, ClothNode *n1) : Edge(n0, n1)
  {
  }

  double ClothAR_size();

  /* Get ClothVert of edge whose node matches n[edge_node] */
  ClothVert *getVert(int face_side, int edge_node);
  /* Get ClothVert of adj_f[face_side] that is not part of this edge */
  ClothVert *getOtherVertOfFace(int face_side);

  bool split(EditedElements &r_ee);
  /* collapse Edge from remove_index to the other Vert/Node, thus
   * removing the Node n[remove_index] */
  bool collapse(int remove_index, EditedElements &r_ee);
  bool flip(EditedElements &r_ee);
};

/* Stores the ClothFace data */
/* Only triangles */
class ClothFace : public Face {
 public:
  double area;
  Mat3x3 dm, dm_inv; /* required data for remeshing step */

  ClothFace() : Face(), area(0.0f)
  {
    type = PRIMITIVE_FACE;
  }

  ClothFace(ClothVert *v0, ClothVert *v1, ClothVert *v2) : Face(v0, v1, v2)
  {
    type = PRIMITIVE_FACE;
  }
};

/* Stores the overall ClothMesh data */
class ClothMesh : public Mesh {
 private:
 public:
  ClothMesh() : Mesh(), coeff_friction(0)
  {
  }

  ClothMesh(const string &filename) : coeff_friction(0)
  {
    loadObj(filename);
  }

  ClothMesh(const string &filename, Shader *shader) : Mesh(shader), coeff_friction(0)
  {
    loadObj(filename);
  }

  ClothMesh(const string &filename, Vec3 pos) : Mesh(pos), coeff_friction(0)
  {
    loadObj(filename);
  }

  ClothMesh(const string &filename, Vec3 pos, Shader *shader)
      : Mesh(pos, shader), coeff_friction(0)
  {
    loadObj(filename);
  }

  ClothMesh(const string &filename, Vec3 pos, Vec3 scale) : Mesh(pos, scale), coeff_friction(0)
  {
    loadObj(filename);
  }

  ClothMesh(const string &filename, Vec3 pos, Vec3 scale, Shader *shader)
      : Mesh(pos, scale, shader), coeff_friction(0)
  {
    loadObj(filename);
  }

  EigenSparseMatrix mass_matrix;     /* TODO(ish): initialize this */
  EigenSparseMatrix identity_matrix; /* TODO(ish): initialize this */
  double coeff_friction;

  void add(ClothVert *vert);
  void add(ClothNode *node);
  void add(ClothEdge *edge);
  void add(ClothFace *face);

  void remove(ClothVert *vert);
  void remove(ClothNode *node);
  void remove(ClothEdge *edge);
  void remove(ClothFace *face);

  bool exists(const ClothEdge *edge);

  void loadObj(const string &file);

  void applyTransformation();
  void unapplyTransformation();

  void updateBVH();

  void setCoeffFriction(double coeff_friction)
  {
    this->coeff_friction = coeff_friction;
  }

  void updateFaceNormals();

  void drawVelocity(glm::mat4 &projection, glm::mat4 &view);

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
