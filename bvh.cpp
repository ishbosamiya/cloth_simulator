#include "bvh.hpp"

const float bvhtree_kdop_axes[13][3] = {
    {1.0, 0, 0},
    {0, 1.0, 0},
    {0, 0, 1.0},
    {1.0, 1.0, 1.0},
    {1.0, -1.0, 1.0},
    {1.0, 1.0, -1.0},
    {1.0, -1.0, -1.0},
    {1.0, 1.0, 0},
    {1.0, 0, 1.0},
    {0, 1.0, 1.0},
    {1.0, -1.0, 0},
    {1.0, 0, -1.0},
    {0, 1.0, -1.0},
};

/* This functions returns the number of branches needed to have the requested number of leafs. */
static int implicit_needed_branches(int tree_type, int leafs)
{
  return max(1, (leafs + tree_type - 3) / (tree_type - 1));
}

BVHTree *BVHTree_new(int maxsize, float epsilon, char tree_type, char axis)
{
  BVHTree *tree;
  int numnodes, i;

  assert(tree_type >= 2 && tree_type <= MAX_TREETYPE);

  tree = new BVHTree;

  /* tree epsilon must be >= FLT_EPSILON
   * so that tangent rays can still hit a bounding volume..
   * this bug would show up when casting a ray aligned with a kdop-axis
   * and with an edge of 2 faces */
  epsilon = max(__FLT_EPSILON__, epsilon);

  if (tree) {
    tree->epsilon = epsilon;
    tree->tree_type = tree_type;
    tree->axis = axis;

    if (axis == 26) {
      tree->start_axis = 0;
      tree->stop_axis = 13;
    }
    else if (axis == 18) {
      tree->start_axis = 7;
      tree->stop_axis = 13;
    }
    else if (axis == 14) {
      tree->start_axis = 0;
      tree->stop_axis = 7;
    }
    else if (axis == 8) { /* AABB */
      tree->start_axis = 0;
      tree->stop_axis = 4;
    }
    else if (axis == 6) { /* OBB */
      tree->start_axis = 0;
      tree->stop_axis = 3;
    }
    else {
      /* should never happen! */
      assert(0);

      BVHTree_free(tree);
      return NULL;
    }

    /* Allocate arrays */
    numnodes = maxsize + implicit_needed_branches(tree_type, maxsize) + tree_type;

    /* Must set them all to NULL/0 to avoid junk value leading to
     * errors, this is done by the "[]();" () part of the "new" operator */
    tree->nodes = new BVHNode *[numnodes]();
    tree->nodebv = new float[axis * numnodes]();
    tree->nodechild = new BVHNode *[tree_type * numnodes]();
    tree->nodearray = new BVHNode[numnodes]();

    if ((!tree->nodes) || (!tree->nodebv) || (!tree->nodechild) || (!tree->nodearray)) {
      BVHTree_free(tree);
      return NULL;
    }

    /* link the dynamic bv and child links */
    for (i = 0; i < numnodes; i++) {
      tree->nodearray[i].bv = &tree->nodebv[i * axis];
      tree->nodearray[i].children = &tree->nodechild[i * tree_type];
    }
  }
  return tree;
}

void BVHTree_free(BVHTree *tree)
{
  if (tree) {
    if (tree->nodes) {
      delete[] tree->nodes;
      tree->nodes = NULL;
    }
    if (tree->nodearray) {
      delete[] tree->nodearray;
      tree->nodearray = NULL;
    }
    if (tree->nodebv) {
      delete[] tree->nodebv;
      tree->nodebv = NULL;
    }
    if (tree->nodechild) {
      delete[] tree->nodechild;
      tree->nodechild = NULL;
    }
    delete tree;
    tree = NULL;
  }
}

struct BVHBuildHelper {
  int tree_type;
  int totleafs;

  /** Min number of leafs that are archievable from a node at depth N */
  int leafs_per_child[32];
  /** Number of nodes at depth N (tree_type^N) */
  int branches_on_level[32];

  /** Number of leafs that are placed on the level that is not 100% filled */
  int remain_leafs;
};

static void build_implicit_tree_helper(const BVHTree *tree, BVHBuildHelper *data)
{
  int depth = 0;
  int remain;
  int nnodes;

  data->totleafs = tree->totleaf;
  data->tree_type = tree->tree_type;

  /* Calculate the smallest tree_type^n such that tree_type^n >= num_leafs */
  for (data->leafs_per_child[0] = 1; data->leafs_per_child[0] < data->totleafs;
       data->leafs_per_child[0] *= data->tree_type) {
    /* pass */
  }

  data->branches_on_level[0] = 1;

  for (depth = 1; (depth < 32) && data->leafs_per_child[depth - 1]; depth++) {
    data->branches_on_level[depth] = data->branches_on_level[depth - 1] * data->tree_type;
    data->leafs_per_child[depth] = data->leafs_per_child[depth - 1] / data->tree_type;
  }

  remain = data->totleafs - data->leafs_per_child[1];
  nnodes = (remain + data->tree_type - 2) / (data->tree_type - 1);
  data->remain_leafs = remain + nnodes;
}

static void node_minmax_init(const BVHTree *tree, BVHNode *node)
{
  axis_t axis_iter;
  float(*bv)[2] = (float(*)[2])node->bv;

  for (axis_iter = tree->start_axis; axis_iter != tree->stop_axis; axis_iter++) {
    bv[axis_iter][0] = numeric_limits<float>::max();
    bv[axis_iter][1] = -numeric_limits<float>::max();
  }
}

/**
 * \note depends on the fact that the BVH's for each face is already built
 */
static void refit_kdop_hull(const BVHTree *tree, BVHNode *node, int start, int end)
{
  float newmin, newmax;
  float *__restrict bv = node->bv;
  int j;
  axis_t axis_iter;

  node_minmax_init(tree, node);

  for (j = start; j < end; j++) {
    float *__restrict node_bv = tree->nodes[j]->bv;

    /* for all Axes. */
    for (axis_iter = tree->start_axis; axis_iter < tree->stop_axis; axis_iter++) {
      newmin = node_bv[(2 * axis_iter)];
      if ((newmin < bv[(2 * axis_iter)])) {
        bv[(2 * axis_iter)] = newmin;
      }

      newmax = node_bv[(2 * axis_iter) + 1];
      if ((newmax > bv[(2 * axis_iter) + 1])) {
        bv[(2 * axis_iter) + 1] = newmax;
      }
    }
  }
}

/**
 * only supports x,y,z axis in the moment
 * but we should use a plain and simple function here for speed sake */
static char get_largest_axis(const float *bv)
{
  float middle_point[3];

  middle_point[0] = (bv[1]) - (bv[0]); /* x axis */
  middle_point[1] = (bv[3]) - (bv[2]); /* y axis */
  middle_point[2] = (bv[5]) - (bv[4]); /* z axis */
  if (middle_point[0] > middle_point[1]) {
    if (middle_point[0] > middle_point[2]) {
      return 1; /* max x axis */
    }
    else {
      return 5; /* max z axis */
    }
  }
  else {
    if (middle_point[1] > middle_point[2]) {
      return 3; /* max y axis */
    }
    else {
      return 5; /* max z axis */
    }
  }
}

/**
 * Return the min index of all the leafs achievable with the given branch.
 */
static int implicit_leafs_index(const BVHBuildHelper *data, const int depth, const int child_index)
{
  int min_leaf_index = child_index * data->leafs_per_child[depth - 1];
  if (min_leaf_index <= data->remain_leafs) {
    return min_leaf_index;
  }
  else if (data->leafs_per_child[depth]) {
    return data->totleafs -
           (data->branches_on_level[depth - 1] - child_index) * data->leafs_per_child[depth];
  }
  else {
    return data->remain_leafs;
  }
}

static int bvh_partition(BVHNode **a, int lo, int hi, BVHNode *x, int axis)
{
  int i = lo, j = hi;
  while (1) {
    while (a[i]->bv[axis] < x->bv[axis]) {
      i++;
    }
    j--;
    while (x->bv[axis] < a[j]->bv[axis]) {
      j--;
    }
    if (!(i < j)) {
      return i;
    }
    BVHNode *temp = a[i];
    a[i] = a[j];
    a[j] = temp;
    i++;
  }
}

/* returns Sortable */
static BVHNode *bvh_medianof3(BVHNode **a, int lo, int mid, int hi, int axis)
{
  if ((a[mid])->bv[axis] < (a[lo])->bv[axis]) {
    if ((a[hi])->bv[axis] < (a[mid])->bv[axis]) {
      return a[mid];
    }
    else {
      if ((a[hi])->bv[axis] < (a[lo])->bv[axis]) {
        return a[hi];
      }
      else {
        return a[lo];
      }
    }
  }
  else {
    if ((a[hi])->bv[axis] < (a[mid])->bv[axis]) {
      if ((a[hi])->bv[axis] < (a[lo])->bv[axis]) {
        return a[lo];
      }
      else {
        return a[hi];
      }
    }
    else {
      return a[mid];
    }
  }
}

/**
 * Insertion sort algorithm
 */
static void bvh_insertionsort(BVHNode **a, int lo, int hi, int axis)
{
  int i, j;
  BVHNode *t;
  for (i = lo; i < hi; i++) {
    j = i;
    t = a[i];
    while ((j != lo) && (t->bv[axis] < (a[j - 1])->bv[axis])) {
      a[j] = a[j - 1];
      j--;
    }
    a[j] = t;
  }
}

/**
 * \note after a call to this function you can expect one of:
 * - every node to left of a[n] are smaller or equal to it
 * - every node to the right of a[n] are greater or equal to it */
static void partition_nth_element(BVHNode **a, int begin, int end, const int n, const int axis)
{
  while (end - begin > 3) {
    const int cut = bvh_partition(
        a, begin, end, bvh_medianof3(a, begin, (begin + end) / 2, end - 1, axis), axis);
    if (cut <= n) {
      begin = cut;
    }
    else {
      end = cut;
    }
  }
  bvh_insertionsort(a, begin, end, axis);
}

/**
 * This function handles the problem of "sorting" the leafs (along the split_axis).
 *
 * It arranges the elements in the given partitions such that:
 * - any element in partition N is less or equal to any element in partition N+1.
 * - if all elements are different all partition will get the same subset of elements
 *   as if the array was sorted.
 *
 * partition P is described as the elements in the range ( nth[P], nth[P+1] ]
 *
 * TODO: This can be optimized a bit by doing a specialized nth_element instead of K nth_elements
 */
static void split_leafs(BVHNode **leafs_array,
                        const int nth[],
                        const int partitions,
                        const int split_axis)
{
  for (int i = 0; i < partitions - 1; i++) {
    if (nth[i] >= nth[partitions]) {
      break;
    }

    partition_nth_element(leafs_array, nth[i], nth[partitions], nth[i + 1], split_axis);
  }
}

struct BVHDivNodesData {
  const BVHTree *tree;
  BVHNode *branches_array;
  BVHNode **leafs_array;

  int tree_type;
  int tree_offset;

  const BVHBuildHelper *data;

  int depth;
  int i;
  int first_of_next_level;
};

static void non_recursive_bvh_div_nodes_task_cb(BVHDivNodesData *data, const int j)
{
  int k;
  const int parent_level_index = j - data->i;
  BVHNode *parent = &data->branches_array[j];
  int nth_positions[MAX_TREETYPE + 1];
  char split_axis;

  int parent_leafs_begin = implicit_leafs_index(data->data, data->depth, parent_level_index);
  int parent_leafs_end = implicit_leafs_index(data->data, data->depth, parent_level_index + 1);

  /* This calculates the bounding box of this branch
   * and chooses the largest axis as the axis to divide leafs */
  refit_kdop_hull(data->tree, parent, parent_leafs_begin, parent_leafs_end);
  split_axis = get_largest_axis(parent->bv);

  /* Save split axis (this can be used on raytracing to speedup the query time) */
  parent->main_axis = split_axis / 2;

  /* Split the childs along the split_axis, note: its not needed to sort the whole leafs array
   * Only to assure that the elements are partitioned on a way that each child takes the elements
   * it would take in case the whole array was sorted.
   * Split_leafs takes care of that "sort" problem. */
  nth_positions[0] = parent_leafs_begin;
  nth_positions[data->tree_type] = parent_leafs_end;
  for (k = 1; k < data->tree_type; k++) {
    const int child_index = j * data->tree_type + data->tree_offset + k;
    /* child level index */
    const int child_level_index = child_index - data->first_of_next_level;
    nth_positions[k] = implicit_leafs_index(data->data, data->depth + 1, child_level_index);
  }

  split_leafs(data->leafs_array, nth_positions, data->tree_type, split_axis);

  /* Setup children and totnode counters
   * Not really needed but currently most of BVH code
   * relies on having an explicit children structure */
  for (k = 0; k < data->tree_type; k++) {
    const int child_index = j * data->tree_type + data->tree_offset + k;
    /* child level index */
    const int child_level_index = child_index - data->first_of_next_level;

    const int child_leafs_begin = implicit_leafs_index(
        data->data, data->depth + 1, child_level_index);
    const int child_leafs_end = implicit_leafs_index(
        data->data, data->depth + 1, child_level_index + 1);

    if (child_leafs_end - child_leafs_begin > 1) {
      parent->children[k] = &data->branches_array[child_index];
      parent->children[k]->parent = parent;
    }
    else if (child_leafs_end - child_leafs_begin == 1) {
      parent->children[k] = data->leafs_array[child_leafs_begin];
      parent->children[k]->parent = parent;
    }
    else {
      break;
    }
  }
  parent->totnode = (char)k;
}

static void non_recursive_bvh_div_nodes(const BVHTree *tree,
                                        BVHNode *branches_array,
                                        BVHNode **leafs_array,
                                        int num_leafs)
{
  int i;

  const int tree_type = tree->tree_type;
  /* this value is 0 (on binary trees) and negative on the others */
  const int tree_offset = 2 - tree->tree_type;

  const int num_branches = implicit_needed_branches(tree_type, num_leafs);

  BVHBuildHelper data;
  int depth;

  {
    /* set parent from root node to NULL */
    BVHNode *root = &branches_array[1];
    root->parent = NULL;

    /* Most of bvhtree code relies on 1-leaf trees having at least one branch
     * We handle that special case here */
    if (num_leafs == 1) {
      refit_kdop_hull(tree, root, 0, num_leafs);
      root->main_axis = get_largest_axis(root->bv) / 2;
      root->totnode = 1;
      root->children[0] = leafs_array[0];
      root->children[0]->parent = root;
      return;
    }
  }

  build_implicit_tree_helper(tree, &data);

  BVHDivNodesData cb_data;
  cb_data.tree = tree;
  cb_data.branches_array = branches_array;
  cb_data.leafs_array = leafs_array;
  cb_data.tree_type = tree_type;
  cb_data.tree_offset = tree_offset;
  cb_data.data = &data;
  cb_data.first_of_next_level = 0;
  cb_data.depth = 0;
  cb_data.i = 0;

  /* Loop tree levels (log N) loops */
  for (i = 1, depth = 1; i <= num_branches; i = i * tree_type + tree_offset, depth++) {
    const int first_of_next_level = i * tree_type + tree_offset;
    /* index of last branch on this level */
    const int i_stop = min(first_of_next_level, num_branches + 1);

    /* Loop all branches on this level */
    cb_data.first_of_next_level = first_of_next_level;
    cb_data.i = i;
    cb_data.depth = depth;

    /* TODO(ish): This can be made parallel, refer to Blender's code
     * to do this */
    for (int i_task = i; i_task < i_stop; i_task++) {
      non_recursive_bvh_div_nodes_task_cb(&cb_data, i_task);
    }
  }
}

void BVHTree_balance(BVHTree *tree)
{
  BVHNode **leafs_array = tree->nodes;

  /* This function should only be called once
   * (some big bug goes here if its being called more than once per tree) */
  assert(tree->totbranch == 0);

  /* Build the implicit tree */
  non_recursive_bvh_div_nodes(
      tree, tree->nodearray + (tree->totleaf - 1), leafs_array, tree->totleaf);

  /* current code expects the branches to be linked to the nodes array
   * we perform that linkage here */
  tree->totbranch = implicit_needed_branches(tree->tree_type, tree->totleaf);
  for (int i = 0; i < tree->totbranch; i++) {
    tree->nodes[tree->totleaf + i] = &tree->nodearray[tree->totleaf + i];
  }
}

static inline float dot_v3v3(const float a[3], const float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/*
 * BVHTree bounding volumes functions
 */
static void create_kdop_hull(
    const BVHTree *tree, BVHNode *node, const float *co, int numpoints, int moving)
{
  float newminmax;
  float *bv = node->bv;
  int k;
  axis_t axis_iter;

  /* don't init boudings for the moving case */
  if (!moving) {
    node_minmax_init(tree, node);
  }

  for (k = 0; k < numpoints; k++) {
    /* for all Axes. */
    for (axis_iter = tree->start_axis; axis_iter < tree->stop_axis; axis_iter++) {
      newminmax = dot_v3v3(&co[k * 3], bvhtree_kdop_axes[axis_iter]);
      if (newminmax < bv[2 * axis_iter]) {
        bv[2 * axis_iter] = newminmax;
      }
      if (newminmax > bv[(2 * axis_iter) + 1]) {
        bv[(2 * axis_iter) + 1] = newminmax;
      }
    }
  }
}

void BVHTree_insert(BVHTree *tree, int index, const float co[3], int numpoints)
{
  axis_t axis_iter;
  BVHNode *node = NULL;

  /* insert should only possible as long as tree->totbranch is 0 */
  assert(tree->totbranch <= 0);

  node = tree->nodes[tree->totleaf] = &(tree->nodearray[tree->totleaf]);
  tree->totleaf++;

  create_kdop_hull(tree, node, co, numpoints, 0);
  node->index = index;

  /* inflate the bv with some epsilon */
  for (axis_iter = tree->start_axis; axis_iter < tree->stop_axis; axis_iter++) {
    node->bv[(2 * axis_iter)] -= tree->epsilon;     /* minimum */
    node->bv[(2 * axis_iter) + 1] += tree->epsilon; /* maximum */
  }
}

/* call before BLI_bvhtree_update_tree() */
bool BVHTree_update_node(
    BVHTree *tree, int index, const float co[3], const float co_moving[3], int numpoints)
{
  BVHNode *node = NULL;
  axis_t axis_iter;

  /* check if index exists */
  if (index > tree->totleaf) {
    return false;
  }

  node = tree->nodearray + index;

  create_kdop_hull(tree, node, co, numpoints, 0);

  if (co_moving) {
    create_kdop_hull(tree, node, co_moving, numpoints, 1);
  }

  /* inflate the bv with some epsilon */
  for (axis_iter = tree->start_axis; axis_iter < tree->stop_axis; axis_iter++) {
    node->bv[(2 * axis_iter)] -= tree->epsilon;     /* minimum */
    node->bv[(2 * axis_iter) + 1] += tree->epsilon; /* maximum */
  }

  return true;
}

/**
 * bottom-up update of bvh node BV
 * join the children on the parent BV */
static void node_join(BVHTree *tree, BVHNode *node)
{
  int i;
  axis_t axis_iter;

  node_minmax_init(tree, node);

  for (i = 0; i < tree->tree_type; i++) {
    if (node->children[i]) {
      for (axis_iter = tree->start_axis; axis_iter < tree->stop_axis; axis_iter++) {
        /* update minimum */
        if (node->children[i]->bv[(2 * axis_iter)] < node->bv[(2 * axis_iter)]) {
          node->bv[(2 * axis_iter)] = node->children[i]->bv[(2 * axis_iter)];
        }

        /* update maximum */
        if (node->children[i]->bv[(2 * axis_iter) + 1] > node->bv[(2 * axis_iter) + 1]) {
          node->bv[(2 * axis_iter) + 1] = node->children[i]->bv[(2 * axis_iter) + 1];
        }
      }
    }
    else {
      break;
    }
  }
}

/* call BLI_bvhtree_update_node() first for every node/point/triangle */
void BVHTree_update_tree(BVHTree *tree)
{
  /* Update bottom=>top
   * TRICKY: the way we build the tree all the childs have an index greater than the parent
   * This allows us todo a bottom up update by starting on the bigger numbered branch */

  BVHNode **root = tree->nodes + tree->totleaf;
  BVHNode **index = tree->nodes + tree->totleaf + tree->totbranch - 1;

  for (; index >= root; index--) {
    node_join(tree, *index);
  }
}

int BVHTree_overlap_thread_num(const BVHTree *tree)
{
  return min(tree->tree_type, tree->nodes[tree->totleaf]->totnode);
}

/**
 * overlap - is it possible for 2 bv's to collide ?
 */
static bool tree_overlap_test(const BVHNode *node1,
                              const BVHNode *node2,
                              axis_t start_axis,
                              axis_t stop_axis)
{
  const float *bv1 = node1->bv + (start_axis << 1);
  const float *bv2 = node2->bv + (start_axis << 1);
  const float *bv1_end = node1->bv + (stop_axis << 1);

  /* test all axis if min + max overlap */
  for (; bv1 != bv1_end; bv1 += 2, bv2 += 2) {
    if ((bv1[0] > bv2[1]) || (bv2[0] > bv1[1])) {
      return 0;
    }
  }

  return 1;
}

static inline axis_t min_axis(axis_t a, axis_t b)
{
  return (a < b) ? a : b;
}

/**
 * a version of #tree_overlap_traverse_cb that that break on first true return.
 */
static bool tree_overlap_traverse_num(BVHOverlapData_Thread *data_thread,
                                      const BVHNode *node1,
                                      const BVHNode *node2)
{
  BVHOverlapData_Shared *data = data_thread->shared;
  int j;

  if (tree_overlap_test(node1, node2, data->start_axis, data->stop_axis)) {
    /* check if node1 is a leaf */
    if (!node1->totnode) {
      /* check if node2 is a leaf */
      if (!node2->totnode) {
        BVHTreeOverlap overlap;

        if (node1 == node2) {
          return false;
        }

        /* only difference to tree_overlap_traverse! */
        if (!data->callback ||
            data->callback(data->userdata, node1->index, node2->index, data_thread->thread)) {
          /* both leafs, insert overlap! */
          if (data_thread->overlap) {
            overlap.indexA = node1->index;
            overlap.indexB = node2->index;
            data_thread->overlap->push(overlap);
          }
          return (--data_thread->max_interactions) == 0;
        }
      }
      else {
        for (j = 0; j < node2->totnode; j++) {
          if (tree_overlap_traverse_num(data_thread, node1, node2->children[j])) {
            return true;
          }
        }
      }
    }
    else {
      const uint max_interactions = data_thread->max_interactions;
      for (j = 0; j < node1->totnode; j++) {
        if (tree_overlap_traverse_num(data_thread, node1->children[j], node2)) {
          data_thread->max_interactions = max_interactions;
        }
      }
    }
  }
  return false;
}

/**
 * a version of #tree_overlap_traverse that runs a callback to check if the nodes really intersect.
 */
static void tree_overlap_traverse_cb(BVHOverlapData_Thread *data_thread,
                                     const BVHNode *node1,
                                     const BVHNode *node2)
{
  BVHOverlapData_Shared *data = data_thread->shared;
  int j;

  if (tree_overlap_test(node1, node2, data->start_axis, data->stop_axis)) {
    /* check if node1 is a leaf */
    if (!node1->totnode) {
      /* check if node2 is a leaf */
      if (!node2->totnode) {
        BVHTreeOverlap overlap;

        if (node1 == node2) {
          return;
        }

        /* only difference to tree_overlap_traverse! */
        if (data->callback(data->userdata, node1->index, node2->index, data_thread->thread)) {
          /* both leafs, insert overlap! */
          overlap.indexA = node1->index;
          overlap.indexB = node2->index;
          data_thread->overlap->push(overlap);
        }
      }
      else {
        for (j = 0; j < data->tree2->tree_type; j++) {
          if (node2->children[j]) {
            tree_overlap_traverse_cb(data_thread, node1, node2->children[j]);
          }
        }
      }
    }
    else {
      for (j = 0; j < data->tree1->tree_type; j++) {
        if (node1->children[j]) {
          tree_overlap_traverse_cb(data_thread, node1->children[j], node2);
        }
      }
    }
  }
}

static void tree_overlap_traverse(BVHOverlapData_Thread *data_thread,
                                  const BVHNode *node1,
                                  const BVHNode *node2)
{
  BVHOverlapData_Shared *data = data_thread->shared;
  int j;

  if (tree_overlap_test(node1, node2, data->start_axis, data->stop_axis)) {
    /* check if node1 is a leaf */
    if (!node1->totnode) {
      /* check if node2 is a leaf */
      if (!node2->totnode) {
        BVHTreeOverlap overlap;

        if (node1 == node2) {
          return;
        }

        /* both leafs, insert overlap! */
        overlap.indexA = node1->index;
        overlap.indexB = node2->index;
        data_thread->overlap->push(overlap);
      }
      else {
        for (j = 0; j < data->tree2->tree_type; j++) {
          if (node2->children[j]) {
            tree_overlap_traverse(data_thread, node1, node2->children[j]);
          }
        }
      }
    }
    else {
      for (j = 0; j < data->tree1->tree_type; j++) {
        if (node1->children[j]) {
          tree_overlap_traverse(data_thread, node1->children[j], node2);
        }
      }
    }
  }
}

BVHTreeOverlap *BVHTree_overlap_ex(
    const BVHTree *tree1,
    const BVHTree *tree2,
    unsigned int *r_overlap_tot,
    /* optional callback to test the overlap before adding (must be thread-safe!) */
    BVHTree_OverlapCallback callback,
    void *userdata,
    const unsigned int max_interactions,
    const int flag)
{
  bool overlap_pairs = (flag & BVH_OVERLAP_RETURN_PAIRS) != 0;
  /* bool use_threading = (flag & BVH_OVERLAP_USE_THREADING) != 0 && */
  /*                      (tree1->totleaf > KDOPBVH_THREAD_LEAF_THRESHOLD); */
  bool use_threading = false;

  /* `RETURN_PAIRS` was not implemented without `max_interations`. */
  assert(overlap_pairs || max_interactions);

  const int root_node_len = BVHTree_overlap_thread_num(tree1);
  /* const int thread_num = use_threading ? root_node_len : 1; */
  const int thread_num = 1;
  int j;
  size_t total = 0;
  BVHTreeOverlap *overlap = NULL, *to = NULL;
  BVHOverlapData_Shared data_shared;
  BVHOverlapData_Thread data[thread_num];
  axis_t start_axis, stop_axis;

  /* check for compatibility of both trees (can't compare 14-DOP with 18-DOP) */
  if ((tree1->axis != tree2->axis) && (tree1->axis == 14 || tree2->axis == 14) &&
      (tree1->axis == 18 || tree2->axis == 18)) {
    assert(0);
    return NULL;
  }

  const BVHNode *root1 = tree1->nodes[tree1->totleaf];
  const BVHNode *root2 = tree2->nodes[tree2->totleaf];

  start_axis = min_axis(tree1->start_axis, tree2->start_axis);
  stop_axis = min_axis(tree1->stop_axis, tree2->stop_axis);

  /* fast check root nodes for collision before doing big splitting + traversal */
  if (!tree_overlap_test(root1, root2, start_axis, stop_axis)) {
    return NULL;
  }

  data_shared.tree1 = tree1;
  data_shared.tree2 = tree2;
  data_shared.start_axis = start_axis;
  data_shared.stop_axis = stop_axis;

  /* can be NULL */
  data_shared.callback = callback;
  data_shared.userdata = userdata;

  for (j = 0; j < thread_num; j++) {
    /* init BVHOverlapData_Thread */
    data[j].shared = &data_shared;
    data[j].overlap = overlap_pairs ? new stack<BVHTreeOverlap> : NULL;
    data[j].max_interactions = max_interactions;

    /* for callback */
    data[j].thread = j;
  }

  if (use_threading) {
    /* TaskParallelSettings settings; */
    /* BLI_parallel_range_settings_defaults(&settings); */
    /* settings.min_iter_per_thread = 1; */
    /* BLI_task_parallel_range(0, root_node_len, data, bvhtree_overlap_task_cb, &settings); */
  }
  else {
    if (max_interactions) {
      tree_overlap_traverse_num(data, root1, root2);
    }
    else if (callback) {
      tree_overlap_traverse_cb(data, root1, root2);
    }
    else {
      tree_overlap_traverse(data, root1, root2);
    }
  }

  if (overlap_pairs) {
    for (j = 0; j < thread_num; j++) {
      total += data[j].overlap->size();
    }

    to = overlap = new BVHTreeOverlap[total];

    for (j = 0; j < thread_num; j++) {
      unsigned int count = (unsigned int)data[j].overlap->size();
      for (unsigned int stack_count_index = 0; stack_count_index < count; stack_count_index++) {
        to[stack_count_index] = data[j].overlap->top();
        data[j].overlap->pop();
      }
      delete data[j].overlap;
      to += count;
    }
    *r_overlap_tot = (unsigned int)total;
  }

  return overlap;
}

BVHTreeOverlap *BVHTree_overlap(
    const BVHTree *tree1,
    const BVHTree *tree2,
    unsigned int *r_overlap_tot,
    /* optional callback to test the overlap before adding (must be thread-safe!) */
    BVHTree_OverlapCallback callback,
    void *userdata)
{
  /* TODO(ish): add multithreading support */
  return BVHTree_overlap_ex(
      tree1, tree2, r_overlap_tot, callback, userdata, 0, BVH_OVERLAP_RETURN_PAIRS);
}

static void draw_line(const Vec3 &pos1,
                      const Vec4 &col1,
                      const Vec3 &pos2,
                      const Vec4 &col2,
                      const uint pos,
                      const uint col)
{
  immAttr4f(col, col1[0], col1[1], col1[2], col1[3]);
  immVertex3f(pos, pos1[0], pos1[1], pos1[2]);
  immAttr4f(col, col2[0], col2[1], col2[2], col2[3]);
  immVertex3f(pos, pos2[0], pos2[1], pos2[2]);
}

void BVHTree_draw(const BVHTree *tree, glm::mat4 &projection, glm::mat4 &view, Vec4 color)
{
  /* Currently only drawing aabb is supported */
  assert(tree->axis == 8);
  static Shader smooth_shader("shaders/shader_3D_smooth_color.vert",
                              "shaders/shader_3D_smooth_color.frag");
  glm::mat4 model = glm::mat4(1.0);
  smooth_shader.use();
  smooth_shader.setMat4("projection", projection);
  smooth_shader.setMat4("view", view);
  smooth_shader.setMat4("model", model);

  glEnable(GL_LINE_SMOOTH);
  glLineWidth(2);

  GPUVertFormat *format = immVertexFormat();
  uint pos = format->addAttribute("pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
  uint col = format->addAttribute("color", GPU_COMP_F32, 4, GPU_FETCH_FLOAT);

  immBegin(GPU_PRIM_LINES, tree->totleaf * 12, &smooth_shader);

  for (int i = 0; i < tree->totleaf; i++) {
    const BVHNode *node = tree->nodearray + i;
    const float &x0 = node->bv[(2 * 0) + 0];
    const float &x1 = node->bv[(2 * 0) + 1];
    const float &y0 = node->bv[(2 * 1) + 0];
    const float &y1 = node->bv[(2 * 1) + 1];
    const float &z0 = node->bv[(2 * 2) + 0];
    const float &z1 = node->bv[(2 * 2) + 1];

    Vec3 v0(x0, y0, z0);
    Vec3 v1(x1, y0, z0);
    Vec3 v2(x1, y1, z0);
    Vec3 v3(x0, y1, z0);
    Vec3 v4(x0, y0, z1);
    Vec3 v5(x1, y0, z1);
    Vec3 v6(x1, y1, z1);
    Vec3 v7(x0, y1, z1);

    draw_line(v0, color, v1, color, pos, col);
    draw_line(v1, color, v2, color, pos, col);
    draw_line(v2, color, v3, color, pos, col);
    draw_line(v3, color, v0, color, pos, col);

    draw_line(v4, color, v5, color, pos, col);
    draw_line(v5, color, v6, color, pos, col);
    draw_line(v6, color, v7, color, pos, col);
    draw_line(v7, color, v4, color, pos, col);

    draw_line(v0, color, v4, color, pos, col);
    draw_line(v1, color, v5, color, pos, col);
    draw_line(v2, color, v6, color, pos, col);
    draw_line(v3, color, v7, color, pos, col);
  }

  immEnd();
}
