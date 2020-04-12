#ifndef BVH_HPP
#define BVH_HPP

#include <cassert>
#include <limits>
#include <algorithm>

/* Blender's kdopbvh turned into cpp version */

#define MAX_TREETYPE 32

class BVHNode {
 public:
  BVHNode **children;
  BVHNode *parent; /* some user defined traversed need that */
  double *bv;      /* Bounding volume of all nodes, max 13 axis */
  int index;       /* face, edge, vertex index */
  char totnode;    /* how many nodes are used, used for speedup */
  char main_axis;  /* Axis used to split this node */
};

/* keep under 26 bytes for speed purposes */
class BVHTree {
 public:
  BVHNode **nodes;
  BVHNode *nodearray;  /* pre-alloc branch nodes */
  BVHNode **nodechild; /* pre-alloc childs for nodes */
  float *nodebv;       /* pre-alloc bounding-volumes for nodes */
  float epsilon;       /* epslion is used for inflation of the k-dop      */
  int totleaf;         /* leafs */
  int totbranch;
  axis_t start_axis, stop_axis; /* bvhtree_kdop_axes array indices according to axis */
  axis_t axis;                  /* kdop type (6 => OBB, 7 => AABB, ...) */
  char tree_type;               /* type of tree (4 => quadtree) */
};

class BVHTreeOverlap {
 public:
  int indexA;
  int indexB;
};

BVHTree *BVHTree_new(int maxsize, float epsilon, char tree_type, char axis);
void BVHTree_free(BVHTree *tree);

/* construct: first insert points, then call balance */
void BVHTree_insert(BVHTree *tree, int index, const float co[3], int numpoints);
void BVHTree_balance(BVHTree *tree);

/* update: first update points/nodes, then call update_tree to refit the bounding volumes */
bool BVHTree_update_node(
    BVHTree *tree, int index, const float co[3], const float co_moving[3], int numpoints);
void BVHTree_update_tree(BVHTree *tree);

#endif
