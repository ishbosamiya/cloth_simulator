#ifndef PRIMITIVES_HPP
#define PRIMITIVES_HPP

#include <iostream>

#include "opengl_mesh.hpp"
#include "math.hpp"
#include "shader.hpp"

using namespace std;

/* TODO(ish): figure out how to remove excessive pos and scale values,
 * needed because face is also a primitive due to the BVH */
enum PRIMITIVE_TYPE {
  PRIMITIVE = 0,
  PRIMITIVE_MESH = 1,
  PRIMITIVE_CLOTH_MESH = 2,
  PRIMITIVE_FACE = 3,
};

class Primitive {
 protected:
  inline void setShaderModelMatrix()
  {
    shader->use();
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, vec3ToGlmVec3(pos));
    model = glm::scale(model, vec3ToGlmVec3(scale));
    shader->setMat4("model", model);
  }

 public:
  Vec3 pos;            /* Pos of primitive in world space */
  Vec3 scale;          /* Scale of primitive in world space */
  unsigned int index;  /* Index of primitive if part of array, mainly
                        * used for BVHTree, assume is not assigned unless known */
  PRIMITIVE_TYPE type; /* Primitive Type */
  Shader *shader;

  Primitive()
  {
    pos = Vec3(0.0d, 0.0d, 0.0d);
    scale = Vec3(1.0d, 1.0d, 1.0d);
    shader = &defaultShader();
    type = PRIMITIVE;
  }

  Primitive(Shader *shader) : shader(shader)
  {
    pos = Vec3(0.0d, 0.0d, 0.0d);
    scale = Vec3(1.0d, 1.0d, 1.0d);
    type = PRIMITIVE;
  }

  Primitive(Vec3 pos) : pos(pos)
  {
    scale = Vec3(1.0d, 1.0d, 1.0d);
    shader = &defaultShader();
    type = PRIMITIVE;
  }

  Primitive(Vec3 pos, Shader *shader) : pos(pos), shader(shader)
  {
    scale = Vec3(1.0d, 1.0d, 1.0d);
    type = PRIMITIVE;
  }

  Primitive(Vec3 pos, Vec3 scale) : pos(pos), scale(scale)
  {
    shader = &defaultShader();
    type = PRIMITIVE;
  }

  Primitive(Vec3 pos, Vec3 scale, Shader *shader) : pos(pos), scale(scale), shader(shader)
  {
    type = PRIMITIVE;
  }

  void setScale(Vec3 scale)
  {
    this->scale = scale;
  }

  void setPos(Vec3 pos)
  {
    this->pos = pos;
  }

  /* Ensure setShaderModelMatrix() is called within draw() to ensure
     the correct position and scaling is applied while drawing the mesh */
  virtual void draw()
  {
    cout << "warning: reached <Primitive> base class virtual function: " << __func__ << endl;
  }

  /* Ensure position and scaling is taken into account while doing the
     intersection test */
  virtual bool intersectionTest(const Vec3 &p, Vec3 &r_normal, double &r_distance)
  {
    cout << "warning: reached <Primitive> base class virtual function: " << __func__ << endl;
  }
};

class Sphere : public Primitive {
 private:
  void storeAsMesh();

 public:
  double radius;
  GLMesh *mesh;

  Sphere(double radius, Vec3 pos) : Primitive(pos), radius(radius)
  {
    storeAsMesh();
  }

  Sphere(double radius, Vec3 pos, Shader *shader) : Primitive(pos, shader), radius(radius)
  {
    storeAsMesh();
  }

  ~Sphere()
  {
    if (mesh) {
      delete mesh;
    }
  }

  bool intersectionTest(const Vec3 &p, Vec3 &r_normal, double &r_distance)
  {
    Vec3 diff = p - pos;
    r_distance = norm(diff) - (radius * scale[0]) - 1e-6;
    if (r_distance < 0) {
      r_normal = normalize(diff);
      return true;
    }
    return false;
  }

  void draw()
  {
    setShaderModelMatrix();
    mesh->draw();
  }
};

#endif
