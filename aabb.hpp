#ifndef AABB_HPP
#define AABB_HPP

#include <algorithm>

#include "math.hpp"
#include "opengl_mesh.hpp"

using namespace std;

class AABB {
 private:
 public:
  Vec3 min_v;
  Vec3 max_v;

  AABB()
  {
  }
  AABB(Vec3 min_v, Vec3 max_v) : min_v(min_v), max_v(max_v)
  {
  }

  int longestAxis() const
  {
    double a = max_v[0] - min_v[0];
    double b = max_v[1] - min_v[1];
    double c = max_v[2] - min_v[2];

    if (a > b && a > c) {
      return 0;
    }
    else if (b > c) {
      return 1;
    }
    else {
      return 2;
    }
  }

  double area() const
  {
    double a = max_v[0] - min_v[0];
    double b = max_v[1] - min_v[1];
    double c = max_v[2] - min_v[2];

    return 2.0 * (a * b + b * c + c * a);
  }

  void getLines(vector<glm::vec3> &r_pos_box, vector<unsigned int> &r_indices_box)
  {
    double x0 = min_v[0];
    double y0 = min_v[1];
    double z0 = min_v[2];
    double x1 = max_v[0];
    double y1 = max_v[1];
    double z1 = max_v[2];
    glm::vec3 point_0(x0, y0, z0);
    glm::vec3 point_1(x1, y0, z0);
    glm::vec3 point_2(x1, y0, z1);
    glm::vec3 point_3(x0, y0, z1);
    glm::vec3 point_4(x0, y1, z0);
    glm::vec3 point_5(x1, y1, z0);
    glm::vec3 point_6(x1, y1, z1);
    glm::vec3 point_7(x0, y1, z1);

    r_pos_box.push_back(point_0);
    r_pos_box.push_back(point_1);
    r_pos_box.push_back(point_1);
    r_pos_box.push_back(point_2);
    r_pos_box.push_back(point_2);
    r_pos_box.push_back(point_3);
    r_pos_box.push_back(point_3);
    r_pos_box.push_back(point_0);

    r_pos_box.push_back(point_4);
    r_pos_box.push_back(point_5);
    r_pos_box.push_back(point_5);
    r_pos_box.push_back(point_6);
    r_pos_box.push_back(point_6);
    r_pos_box.push_back(point_7);
    r_pos_box.push_back(point_7);
    r_pos_box.push_back(point_4);

    r_pos_box.push_back(point_0);
    r_pos_box.push_back(point_4);
    r_pos_box.push_back(point_1);
    r_pos_box.push_back(point_5);
    r_pos_box.push_back(point_2);
    r_pos_box.push_back(point_6);
    r_pos_box.push_back(point_3);
    r_pos_box.push_back(point_7);

    unsigned int index = r_indices_box.size();
    for (int i = 0; i < 12; i++) {
      r_indices_box.push_back(index++);
      r_indices_box.push_back(index++);
    }
  }
};

AABB surroundingBox(const AABB &box_0, const AABB &box_1);

#endif
