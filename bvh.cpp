#include "bvh.hpp"

BVHNode::BVHNode(Primitive **l, int n)
{
  AABB *boxes = new AABB[n];
  double *left_area = new double[n];
  double *right_area = new double[n];
  AABB main_box;

  bool dummy = l[0]->boundingBox(main_box);
  for (int i = 1; i < n; i++) {
    AABB new_box;
    bool dummy = l[i]->boundingBox(new_box);
    main_box = surroundingBox(new_box, main_box);
  }
  int axis = main_box.longestAxis();

  if (axis == 0) {
    qsort(l, n, sizeof(Primitive *), boxXCompare);
  }
  else if (axis == 1) {
    qsort(l, n, sizeof(Primitive *), boxYCompare);
  }
  else {
    qsort(l, n, sizeof(Primitive *), boxZCompare);
  }

  for (int i = 0; i < n; i++) {
    bool dummy = l[i]->boundingBox(boxes[i]);
  }

  left_area[0] = boxes[0].area();

  AABB left_box = boxes[0];
  for (int i = 1; i < n - 1; i++) {
    left_box = surroundingBox(left_box, boxes[i]);
    left_area[i] = left_box.area();
  }
  right_area[n - 1] = boxes[n - 1].area();

  AABB right_box = boxes[n - 1];
  for (int i = n - 2; i > 0; i--) {
    right_box = surroundingBox(right_box, boxes[i]);
    right_area[i] = right_box.area();
  }

  float min_SAH = FLT_MAX;
  int min_SAH_idx;
  for (int i = 0; i < n - 1; i++) {
    float SAH = i * left_area[i] + (n - i - 1) * right_area[i + 1];
    if (SAH < min_SAH) {
      min_SAH_idx = i;
      min_SAH = SAH;
    }
  }

  if (n == 1) {
    left = right = l[0];
    leaf_node = true;
  }
  else if (n == 2) {
    left = l[0];
    right = l[1];
    leaf_node = true;
  }
  else {
    left = new BVHNode(l, n / 2);
    right = new BVHNode(l + n / 2, n - n / 2);
    leaf_node = false;
  }

  box = main_box;

  delete[] boxes;
  delete[] left_area;
  delete[] right_area;
}

void BVHNode::deleteNode()
{
  if (!leaf_node) {
    static_cast<BVHNode *>(left)->deleteNode();
    static_cast<BVHNode *>(right)->deleteNode();
    delete right;
    delete left;
    left = right = NULL;
  }
}

void BVHNode::getLines(vector<glm::vec3> &r_pos_box, vector<unsigned int> &r_indices_box)
{
  if (leaf_node) {
    box.getLines(r_pos_box, r_indices_box);
  }
  else {
    static_cast<BVHNode *>(left)->getLines(r_pos_box, r_indices_box);
    static_cast<BVHNode *>(right)->getLines(r_pos_box, r_indices_box);
  }
}

void BVHNode::draw(glm::mat4 &projection, glm::mat4 &view)
{
  vector<glm::vec3> pos_box;
  vector<unsigned int> indices_box;
  getLines(pos_box, indices_box);
  static Shader line_shader("shaders/line.vert", "shaders/line.frag");
  line_shader.use();
  line_shader.setMat4("projection", projection);
  line_shader.setMat4("view", view);
  glm::mat4 model = glm::mat4(1.0f);
  line_shader.setMat4("model", model);
  line_shader.setVec4("color", 0.2, 0.4, 0.8, 1.0);
  GLLine line_box(pos_box, indices_box);
  line_box.draw();
}

int boxXCompare(const void *a, const void *b)
{
  AABB box_left, box_right;
  Primitive *ap = *(Primitive **)a;
  Primitive *bp = *(Primitive **)b;
  if (!ap->boundingBox(box_left) || !bp->boundingBox(box_right)) {
    cout << "error: no bounding box in BVHNode constructor" << endl;
  }

  if ((box_left.min_v[0] - box_right.min_v[0]) < 0.0) {
    return -1;
  }
  return 1;
}

int boxYCompare(const void *a, const void *b)
{
  AABB box_left, box_right;
  Primitive *ap = *(Primitive **)a;
  Primitive *bp = *(Primitive **)b;
  if (!ap->boundingBox(box_left) || !bp->boundingBox(box_right)) {
    cout << "error: no bounding box in BVHNode constructor" << endl;
  }

  if ((box_left.min_v[1] - box_right.min_v[1]) < 0.0) {
    return -1;
  }
  return 1;
}

int boxZCompare(const void *a, const void *b)
{
  AABB box_left, box_right;
  Primitive *ap = *(Primitive **)a;
  Primitive *bp = *(Primitive **)b;
  if (!ap->boundingBox(box_left) || !bp->boundingBox(box_right)) {
    cout << "error: no bounding box in BVHNode constructor" << endl;
  }

  if ((box_left.min_v[2] - box_right.min_v[2]) < 0.0) {
    return -1;
  }
  return 1;
}
