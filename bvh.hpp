#ifndef BVH_HPP
#define BVH_HPP

#include <cassert>
#include <limits>
#include <algorithm>
#include <stack>

/* Blender's kdopbvh turned into cpp version */

#define MAX_TREETYPE 32

typedef unsigned char axis_t;

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

enum {
  /* Use a priority queue to process nodes in the optimal order (for slow callbacks) */
  BVH_OVERLAP_RETURN_PAIRS = (1 << 0),
};

/* callback to check if 2 nodes overlap (use thread if intersection results need to be stored) */
typedef bool (*BVHTree_OverlapCallback)(void *userdata, int index_a, int index_b, int thread);

/* avoid duplicating vars in BVHOverlapData_Thread */
struct BVHOverlapData_Shared {
  const BVHTree *tree1, *tree2;
  axis_t start_axis, stop_axis;

  /* use for callbacks */
  BVHTree_OverlapCallback callback;
  void *userdata;
};

struct BVHOverlapData_Thread {
  BVHOverlapData_Shared *shared;
  stack<BVHTreeOverlap> *overlap; /* store BVHTreeOverlap */
  unsigned int max_interactions;
  /* use for callbacks */
  int thread;
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

/**
 * Use to check the total number of threads #BVHTree_overlap will use.
 *
 * \warning Must be the first tree passed to #BVHTree_overlap!
 */
int BVHTree_overlap_thread_num(const BVHTree *tree);

/* collision/overlap: check two trees if they overlap,
 * alloc's *overlap with length of the int return value */
BVHTreeOverlap *BVHTree_overlap_ex(
    const BVHTree *tree1,
    const BVHTree *tree2,
    uint *r_overlap_tot,
    /* optional callback to test the overlap before adding (must be thread-safe!) */
    BVHTree_OverlapCallback callback,
    void *userdata,
    const uint max_interactions,
    const int flag);
BVHTreeOverlap *BVHTree_overlap(const BVHTree *tree1,
                                const BVHTree *tree2,
                                unsigned int *r_overlap_tot,
                                BVHTree_OverlapCallback callback,
                                void *userdata);

#endif
