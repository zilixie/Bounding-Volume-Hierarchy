#include "insert_box_into_box.h"

void insert_box_into_box(
  const BoundingBox & A,
  BoundingBox & B)
{
  Eigen::RowVector3d max_a = A.max_corner;
  Eigen::RowVector3d max_b = B.max_corner;
  Eigen::RowVector3d min_a = A.min_corner;
  Eigen::RowVector3d min_b = B.min_corner;

  B.max_corner.x() = std::fmax(max_a.x(), max_b.x());
  B.max_corner.y() = std::fmax(max_a.y(), max_b.y());
  B.max_corner.z() = std::fmax(max_a.z(), max_b.z());

  B.min_corner.x() = std::fmin(min_a.x(), min_b.x());
  B.min_corner.y() = std::fmin(min_a.y(), min_b.y());
  B.min_corner.z() = std::fmin(min_a.z(), min_b.z());
}
