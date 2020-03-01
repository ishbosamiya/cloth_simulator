#include "aabb.hpp"

AABB surroundingBox(const AABB &box_0, const AABB &box_1)
{
  Vec3 small(min(box_0.min_v[0], box_1.min_v[0]),
             min(box_0.min_v[1], box_1.min_v[1]),
             min(box_0.min_v[2], box_1.min_v[2]));
  Vec3 big(max(box_0.max_v[0], box_1.max_v[0]),
           max(box_0.max_v[1], box_1.max_v[1]),
           max(box_0.max_v[2], box_1.max_v[2]));
  return AABB(small, big);
}
