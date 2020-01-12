#ifndef MESH_HPP
#define MESH_HPP

#include <vector>
#include <cassert>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>

#include "math.hpp"
#include "misc.hpp"

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
class ClothVert {
 public:
  vector<ClothFace *> adj_f; /* reference to adjacent faces wrt to the
                                UV space */
  ClothNode *node;           /*reference to node of vert */
  int index;                 /* position in ClothMesh.verts */
  Vec3 uv;                   /* UV coordinates of vert, stored as Vec3 for easier
                              * calculation */
  Mat3x3 sizing;             /* sizing information for the remeshing
                              * step */

  ClothVert() : node(0), index(-1)
  {
  }
  ClothVert(const Vec3 &u) : uv(u)
  {
  }
};

/* Stores the World Space coordinates */
class ClothNode {
 public:
  vector<ClothVert *> verts; /* This helps in storing all the
                              * references to the UV's of
                              * the Node */
  vector<ClothEdge *> adj_e; /* reference to adjacent edges of the
                              * node */
  int index;                 /* position in ClothMesh.nodes */
  Vec3 x;                    /* world space position of node */
  Vec3 x0;                   /* previous world space position of node */
  Vec3 v;                    /* world space velocity of node */

  ClothNode() : index(-1)
  {
  }
  ClothNode(const Vec3 &x, const Vec3 &v) : x(x), v(v)
  {
  }
};

/* Stores the Edge data */
class ClothEdge {
 public:
  ClothNode *n[2];     /* reference to nodes of edge */
  ClothFace *adj_f[2]; /* reference to adjacent faces of edge */
  int index;           /*position in ClothMesh.edges */

  ClothEdge() : index(-1)
  {
  }

  ClothEdge(ClothNode *n0, ClothNode *n1)
  {
    n[0] = n0;
    n[1] = n1;
  }
};

/* Stores the Face data */
/* Only triangles */
class ClothFace {
 public:
  ClothVert *v[3];     /* reference to verts of the face */
  ClothEdge *adj_e[3]; /* reference to adjacent edges of the face */
  int index;           /* position in ClothMesh.faces */
  Vec3 n;              /* normal */

  double area, mass;
  Mat3x3 dm, dm_inv; /* required data for remeshing step */

  ClothFace() : index(-1), area(0.0f), mass(0.0f)
  {
    for (int i = 0; i < 3; i++) {
      v[i] = NULL;
      adj_e[i] = NULL;
    }
  }

  ClothFace(ClothVert *v0, ClothVert *v1, ClothVert *v2)
  {
    v[0] = v0;
    v[1] = v1;
    v[2] = v2;
  }
};

/* Stores the overall Mesh data */
class ClothMesh {
 public:
  vector<ClothVert *> verts;
  vector<ClothNode *> nodes;
  vector<ClothEdge *> edges;
  vector<ClothFace *> faces;

  void add(ClothVert *vert);
  void add(ClothNode *node);
  void add(ClothEdge *edge);
  void add(ClothFace *face);

  void remove(ClothVert *vert);
  void remove(ClothNode *node);
  void remove(ClothEdge *edge);
  void remove(ClothFace *face);

  void setIndices();

  void loadObj(const string &file);

  void deleteMesh();
  ~ClothMesh()
  {
    deleteMesh();
  }
};

inline ClothEdge *getEdge(const ClothNode *n0, const ClothNode *n1)
{
  for (int i = 0; i < (int)n0->adj_e.size(); i++) {
    ClothEdge *edge = n0->adj_e[i];
    if (edge->n[0] == n1 || edge->n[1] == n1) {
      return edge;
    }
  }
  return NULL;
}

#endif
