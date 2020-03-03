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
  bool decend(BVHNode *other)
  {
    return other->leaf_node || (!leaf_node && (box.area() >= other->box.area()));
  }

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

  void overlap(BVHNode *other, vector<glm::vec3> &r_pos_box, vector<unsigned int> &r_indices_box)
  {
    if (!box.overlap(other->box)) {
      return;
    }
    if (leaf_node && other->leaf_node) {
      /* TODO(ish): primitives collide */
      if (this != other) {
        box.getLines(r_pos_box, r_indices_box);
        other->box.getLines(r_pos_box, r_indices_box);
      }
    }
    else {
      if (decend(other)) {
        static_cast<BVHNode *>(left)->overlap(other, r_pos_box, r_indices_box);
        static_cast<BVHNode *>(right)->overlap(other, r_pos_box, r_indices_box);
      }
      else {
        static_cast<BVHNode *>(other->left)->overlap(this, r_pos_box, r_indices_box);
        static_cast<BVHNode *>(other->right)->overlap(this, r_pos_box, r_indices_box);
      }
    }
  }

  ~BVHNode()
  {
    deleteNode();
  }
};

#endif
