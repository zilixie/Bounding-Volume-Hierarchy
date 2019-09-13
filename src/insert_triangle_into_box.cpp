#include "insert_triangle_into_box.h"

void insert_triangle_into_box(
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  BoundingBox & B)
{
  double tmp;
  tmp = ((a.x() < b.x()) ? b.x():a.x());
  double max_x = ((tmp < c.x()) ? c.x():tmp);
  tmp = ((a.y() < b.y()) ? b.y():a.y());
  double max_y = ((tmp < c.y()) ? c.y():tmp);
  tmp = ((a.z() < b.z()) ? b.z():a.z());
  double max_z = ((tmp < c.z()) ? c.z():tmp);

  tmp = ((a.x() < b.x()) ? a.x():b.x());
  double min_x = ((tmp < c.x()) ? tmp:c.x());
  tmp = ((a.y() < b.y()) ? a.y():b.y());
  double min_y = ((tmp < c.y()) ? tmp:c.y());
  tmp = ((a.z() < b.z()) ? a.z():b.z());
  double min_z = ((tmp < c.z()) ? tmp:c.z());

  B.max_corner = {max_x, max_y, max_z};
  B.min_corner = {min_x, min_y, min_z};
}


