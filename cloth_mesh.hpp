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
  Vec3 x0; /* previous world space position of node */
  Vec3 v;  /* world space velocity of node */

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
  }

  ClothFace(ClothVert *v0, ClothVert *v1, ClothVert *v2) : Face(v0, v1, v2)
  {
  }
};

/* Stores the overall ClothMesh data */
class ClothMesh : public Mesh {
 private:
 public:
  ClothMesh() : Mesh()
  {
  }

  ClothMesh(const string &filename) : Mesh(filename)
  {
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

  ~ClothMesh()
  {
  }
};

#define PI 3.141592653
class Sphere {
 public:
  double radius;
  Vec3 position;
  GLMesh *mesh;

  Sphere(double radius, Vec3 position) : radius(radius), position(position)
  {
    storeAsMesh();
  }

  void storeAsMesh()
  {
    vector<GLVertex> verts;
    vector<unsigned int> indices;
    int stacks = 16;
    int slices = 32;
    int indices_count = 0;
    for (int t = 0; t < stacks; t++) {
      double theta1 = ((double)(t) / stacks) * PI;
      double theta2 = ((double)(t + 1) / stacks) * PI;
      for (int p = 0; p < slices; p++) {
        double phi1 = ((double)(p) / slices) * 2 * PI;
        double phi2 = ((double)(p + 1) / slices) * 2 * PI;

        GLVertex v1;
        GLVertex v2;
        GLVertex v3;
        GLVertex v4;

        v1.x = glm::vec3(radius * sin(theta1) * cos(phi1),
                         radius * sin(theta1) * sin(phi1),
                         radius * cos(theta1)) +
               glm::vec3(position[0], position[1], position[2]);

        v2.x = glm::vec3(radius * sin(theta1) * cos(phi2),
                         radius * sin(theta1) * sin(phi2),
                         radius * cos(theta1)) +
               glm::vec3(position[0], position[1], position[2]);

        v3.x = glm::vec3(radius * sin(theta2) * cos(phi2),
                         radius * sin(theta2) * sin(phi2),
                         radius * cos(theta2)) +
               glm::vec3(position[0], position[1], position[2]);

        v4.x = glm::vec3(radius * sin(theta2) * cos(phi1),
                         radius * sin(theta2) * sin(phi1),
                         radius * cos(theta2)) +
               glm::vec3(position[0], position[1], position[2]);

        verts.push_back(v1);
        verts.push_back(v2);
        verts.push_back(v3);
        verts.push_back(v4);

        if (t == 0) {
          indices.push_back(indices_count + 0);
          indices.push_back(indices_count + 2);
          indices.push_back(indices_count + 3);
        }
        else if (t + 1 == stacks) {
          indices.push_back(indices_count + 2);
          indices.push_back(indices_count + 0);
          indices.push_back(indices_count + 1);
        }
        else {
          indices.push_back(indices_count + 0);
          indices.push_back(indices_count + 1);
          indices.push_back(indices_count + 3);

          indices.push_back(indices_count + 1);
          indices.push_back(indices_count + 2);
          indices.push_back(indices_count + 3);
        }
        indices_count += 4;
      }
    }

    vector<glm::vec3> normals;
    normals.resize(verts.size());
    for (int i = 0; i < normals.size(); i++) {
      normals[i] = glm::vec3(0, 0, 0);
    }

    for (int i = 0; i < indices.size(); i = i + 3) {
      GLVertex v0 = verts[indices[i + 0]];
      GLVertex v1 = verts[indices[i + 1]];
      GLVertex v2 = verts[indices[i + 2]];

      glm::vec3 n = glm::cross(v2.x - v0.x, v1.x - v0.x);

      normals[indices[i + 0]] += n;
      normals[indices[i + 1]] += n;
      normals[indices[i + 2]] += n;
    }

    for (int i = 0; i < verts.size(); i++) {
      verts[i].n = glm::normalize(normals[i]);
    }

    mesh = new GLMesh(verts, indices);
  }

  ~Sphere()
  {
    if (mesh) {
      delete mesh;
    }
  }

  bool intersectionTest(const Vec3 &p, Vec3 &r_normal, double &r_distance)
  {
    Vec3 diff = p - position;
    r_distance = norm(diff) - radius - 1e-6;
    if (r_distance < 0) {
      r_normal = normalize(diff);
      return true;
    }
    return false;
  }

  void draw()
  {
    mesh->draw();
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
