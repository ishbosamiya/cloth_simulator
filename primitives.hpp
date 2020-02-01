#ifndef PRIMITIVES_HPP
#define PRIMITIVES_HPP

#include "opengl_mesh.hpp"
#include "math.hpp"

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

#endif
