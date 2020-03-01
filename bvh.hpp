#ifndef BVH_HPP
#define BVH_HPP

#include <iostream>
#include <algorithm>
#include <limits>
#include <cassert>

#include "aabb.hpp"
#include "math.hpp"
#include "primitives.hpp"
#include "opengl_mesh.hpp"

using namespace std;

int boxXCompare(const void *a, const void *b);
int boxYCompare(const void *a, const void *b);
int boxZCompare(const void *a, const void *b);

class BVHNode : Primitive {
 private:
  void deleteNode();
  void getLines(vector<glm::vec3> &r_pos_box, vector<unsigned int> &r_indices_box);

 public:
  Primitive *left;
  Primitive *right;
  AABB box;
  bool leaf_node;

  BVHNode()
  {
  }
  BVHNode(Primitive **l, int n);

  void draw(glm::mat4 &projection, glm::mat4 &view);

  ~BVHNode()
  {
    deleteNode();
  }
};

#endif
