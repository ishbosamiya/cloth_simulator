#include "bvh.hpp"

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
  epsilon = max(FLT_EPSILON, epsilon);

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

    tree->nodes = new (BVHNode *)[numnodes];
    tree->nodebv = new float[axis * numnodes];
    tree->nodechild = new (BVHNode *)[tree_type * numnodes];
    tree->nodearray = new BVHNode[numnodes];

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
      delete tree->nodes;
      tree->nodes = NULL;
    }
    if (tree->nodearray) {
      delete tree->nodearray;
      tree->nodearray = NULL;
    }
    if (tree->nodebv) {
      delete tree->nodebv;
      tree->nodebv = NULL;
    }
    if (tree->nodechild) {
      delete tree->nodechild;
      tree->nodechild = NULL;
    }
    delete tree;
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
    bv[axis_iter][0] = FLT_MAX;
    bv[axis_iter][1] = -FLT_MAX;
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

  BVHDivNodesData cb_data = {
      .tree = tree,
      .branches_array = branches_array,
      .leafs_array = leafs_array,
      .tree_type = tree_type,
      .tree_offset = tree_offset,
      .data = &data,
      .first_of_next_level = 0,
      .depth = 0,
      .i = 0,
  };

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
