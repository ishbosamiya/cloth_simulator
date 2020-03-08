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

class BVHTreeOverlapResult {
 public:
  unsigned int indexA;
  unsigned int indexB;
  BVHTreeOverlapResult(unsigned int indexA, unsigned int indexB) : indexA(indexA), indexB(indexB)
  {
  }
};

typedef bool (*BVH_collisionCheck)(Primitive *, Primitive *, void *user_data);

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

  template<typename T>
  void overlap(BVHNode *other,
               vector<glm::vec3> &r_pos_box,
               vector<unsigned int> &r_indices_box,
               BVH_collisionCheck collision_check,
               T *user_data,
               vector<BVHTreeOverlapResult> &r_result)
  {
    if (!box.overlap(other->box)) {
      return;
    }
    if (leaf_node && other->leaf_node) {
      if (collision_check) {
        if (collision_check(left, other->left, user_data)) {
          r_result.push_back(BVHTreeOverlapResult(left->index, other->left->index));
        }
        if (collision_check(left, other->right, user_data)) {
          r_result.push_back(BVHTreeOverlapResult(left->index, other->right->index));
        }
        if (collision_check(right, other->left, user_data)) {
          r_result.push_back(BVHTreeOverlapResult(right->index, other->left->index));
        }
        if (collision_check(right, other->right, user_data)) {
          r_result.push_back(BVHTreeOverlapResult(right->index, other->right->index));
        }
      }
      else {
        if (left->type != 0) {
          r_result.push_back(BVHTreeOverlapResult(left->index, other->left->index));
          r_result.push_back(BVHTreeOverlapResult(left->index, other->right->index));
        }
        if (right->type != 0) {
          r_result.push_back(BVHTreeOverlapResult(right->index, other->left->index));
          r_result.push_back(BVHTreeOverlapResult(right->index, other->right->index));
        }
        /* cout << "left->type: " << left->type << " right->type: " << right->type */
        /*      << " other->left->type: " << other->left->type */
        /*      << " other->right->type: " << other->right->type << endl; */
        /* cout << "left->index: " << left->index << " right->index: " << right->index */
        /*      << " other->left->index: " << other->left->index */
        /*      << " other->right->index: " << other->right->index << endl; */
      }

      if (this != other) {
        box.getLines(r_pos_box, r_indices_box);
        other->box.getLines(r_pos_box, r_indices_box);
      }
    }
    else {
      if (decend(other)) {
        static_cast<BVHNode *>(left)->overlap(
            other, r_pos_box, r_indices_box, collision_check, user_data, r_result);
        static_cast<BVHNode *>(right)->overlap(
            other, r_pos_box, r_indices_box, collision_check, user_data, r_result);
      }
      else {
        static_cast<BVHNode *>(other->left)
            ->overlap(this, r_pos_box, r_indices_box, collision_check, user_data, r_result);
        static_cast<BVHNode *>(other->right)
            ->overlap(this, r_pos_box, r_indices_box, collision_check, user_data, r_result);
      }
    }
  }

  ~BVHNode()
  {
    deleteNode();
  }
};

#endif
