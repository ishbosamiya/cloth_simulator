#ifndef MESH_HPP
#define MESH_HPP

#include <vector>
#include <cassert>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <limits>

#include "math.hpp"
#include "misc.hpp"
#include "opengl_mesh.hpp"
#include "primitives.hpp"
#include "bvh.hpp"
#include "gpu_immediate.hpp"

using namespace std;

class Vert;
class Node;
class Edge;
class Face;
class Mesh;
class EditedElements;

/* Important to note that Node is the world space vertex which can
 * have UV space coordinates thus is split into Node and
 * Vert.
 * Generally Node would be called a vertex */

/* Stores the UV information and corresponding World Space Node */
class Vert {
 public:
  vector<Face *> adj_f; /* reference to adjacent faces wrt to the
                                UV space */
  Node *node;           /*reference to node of vert */
  int index;            /* position in Mesh.verts */
  Vec3 uv;              /* UV coordinates of vert, stored as Vec3 for easier
                         * calculation */

  Vert() : node(0), index(-1)
  {
  }
  Vert(const Vec3 &uv) : uv(uv)
  {
  }
};

/* Stores the World Space coordinates */
class Node {
 public:
  vector<Vert *> verts; /* This helps in storing all the
                         * references to the UV's of
                         * the Node */
  vector<Edge *> adj_e; /* reference to adjacent edges of the
                         * node */
  int index;            /* position in Mesh.nodes */
  Vec3 x;               /* world space position of node */
  Vec3 n;               /* world space normal */

  Node() : index(-1)
  {
  }
  Node(const Vec3 &x) : x(x)
  {
  }
  Node(const Vec3 &x, const Vec3 &n) : x(x), n(n)
  {
  }

  virtual ~Node()
  {
  }
};

/* Stores the Edge data */
class Edge {
 public:
  Node *n[2];     /* reference to nodes of edge */
  Face *adj_f[2]; /* reference to adjacent faces of edge */
  int index;      /*position in Mesh.edges */

  Edge() : index(-1)
  {
  }

  Edge(Node *n0, Node *n1)
  {
    n[0] = n0;
    n[1] = n1;
  }

  /* Get Vert of edge whose node matches n[edge_node] */
  Vert *getVert(int face_side, int edge_node);
  /* Get Vert of adj_f[face_side] that is not part of this edge */
  Vert *getOtherVertOfFace(int face_side);

  bool split(EditedElements &r_ee);
};

/* Stores the Face data */
/* Only triangles */
class Face : public Primitive {
 public:
  Vert *v[3];              /* reference to verts of the face */
  Edge *adj_e[3];          /* reference to adjacent edges of the face */
  /* unsigned int index */ /* position in Mesh.faces, is in Primitive */
  Vec3 n;                  /* normal */

  Face()
  {
    for (int i = 0; i < 3; i++) {
      v[i] = NULL;
      adj_e[i] = NULL;
    }
  }

  Face(Vert *v0, Vert *v1, Vert *v2)
  {
    v[0] = v0;
    v[1] = v1;
    v[2] = v2;
  }
};

/* Stores the overall Mesh data */
class Mesh : public Primitive {
 private:
  void setIndices();
  GLMesh convertToGLMesh();
  void deleteMesh();

 public:
  Mesh()
  {
  }

  Mesh(const string &filename) : bvh(NULL)
  {
    Mesh::loadObj(filename);
  }

  Mesh(const string &filename, Shader *shader) : Primitive(shader), bvh(NULL)
  {
    Mesh::loadObj(filename);
  }

  Mesh(const string &filename, Vec3 pos) : Primitive(pos), bvh(NULL)
  {
    Mesh::loadObj(filename);
  }

  Mesh(const string &filename, Vec3 pos, Shader *shader) : Primitive(pos, shader), bvh(NULL)
  {
    Mesh::loadObj(filename);
  }

  Mesh(const string &filename, Vec3 pos, Vec3 scale) : Primitive(pos, scale), bvh(NULL)
  {
    Mesh::loadObj(filename);
  }

  Mesh(const string &filename, Vec3 pos, Vec3 scale, Shader *shader)
      : Primitive(pos, scale, shader), bvh(NULL)
  {
    Mesh::loadObj(filename);
  }

  Mesh(Shader *shader) : Primitive(shader), bvh(NULL)
  {
  }

  Mesh(Vec3 pos) : Primitive(pos), bvh(NULL)
  {
  }

  Mesh(Vec3 pos, Shader *shader) : Primitive(pos, shader), bvh(NULL)
  {
  }

  Mesh(Vec3 pos, Vec3 scale) : Primitive(pos, scale), bvh(NULL)
  {
  }

  Mesh(Vec3 pos, Vec3 scale, Shader *shader) : Primitive(pos, scale, shader), bvh(NULL)
  {
  }

  vector<Vert *> verts;
  vector<Node *> nodes;
  vector<Edge *> edges;
  vector<Face *> faces;
  BVHTree *bvh; /* The BVH for the mesh */

  virtual void add(Vert *vert);
  virtual void add(Node *node);
  virtual void add(Edge *edge);
  virtual void add(Face *face);

  virtual void remove(Vert *vert);
  virtual void remove(Node *node);
  virtual void remove(Edge *edge);
  virtual void remove(Face *face);

  virtual void loadObj(const string &file);
  void saveObj(const string &filename);

  void shadeSmooth();

  virtual void draw();
  void drawWireframe(glm::mat4 projection, glm::mat4 view, Vec4 color);

  virtual bool intersectionTest(const Vec3 &p, Vec3 &r_normal, double &r_distance)
  {
    return false;
  }

  virtual void applyTransformation();
  virtual void unapplyTransformation();

  void buildBVH(float epsilon = 0.01);
  void updateBVH();
  void deleteBVH();
  void drawBVH(glm::mat4 &projection, glm::mat4 &view, Vec4 color, int draw_level)
  {
    if (bvh) {
      BVHTree_draw(bvh, projection, view, color, draw_level);
    }
  }

  void updateFaceNormals();

  ~Mesh()
  {
    deleteMesh();
  }
};

class EditedElements {
 public:
  vector<Face *> removed_faces;
  vector<Edge *> removed_edges;
  vector<Node *> removed_nodes;
  vector<Vert *> removed_verts;
  vector<Face *> added_faces;
  vector<Edge *> added_edges;
  vector<Node *> added_nodes;
  vector<Vert *> added_verts;

  inline void remove(Face *face)
  {
    removed_faces.push_back(face);
  }
  inline void remove(Edge *edge)
  {
    removed_edges.push_back(edge);
  }
  inline void remove(Node *node)
  {
    removed_nodes.push_back(node);
  }
  inline void remove(Vert *vert)
  {
    removed_verts.push_back(vert);
  }

  inline void add(Face *face)
  {
    added_faces.push_back(face);
  }
  inline void add(Edge *edge)
  {
    added_edges.push_back(edge);
  }
  inline void add(Node *node)
  {
    added_nodes.push_back(node);
  }
  inline void add(Vert *vert)
  {
    added_verts.push_back(vert);
  }

  void apply(Mesh &mesh);
};

inline Edge *getEdge(const Node *n0, const Node *n1)
{
  for (int i = 0; i < (int)n0->adj_e.size(); i++) {
    Edge *edge = n0->adj_e[i];
    if (edge->n[0] == n1 || edge->n[1] == n1) {
      return edge;
    }
  }
  return NULL;
}

#endif
