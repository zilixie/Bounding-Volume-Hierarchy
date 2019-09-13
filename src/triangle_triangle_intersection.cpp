#include "triangle_triangle_intersection.h"
#include "ray_intersect_triangle.h"

bool triangle_triangle_intersection(
  const Eigen::RowVector3d & A0,
  const Eigen::RowVector3d & A1,
  const Eigen::RowVector3d & A2,
  const Eigen::RowVector3d & B0,
  const Eigen::RowVector3d & B1,
  const Eigen::RowVector3d & B2)
{
  Eigen::RowVector3d a0_to_a1 = A1 - A0;
  Eigen::RowVector3d a1_to_a2 = A2 - A1;
  Eigen::RowVector3d a2_to_a0 = A0 - A2;
  Eigen::RowVector3d b0_to_b1 = B1 - B0;
  Eigen::RowVector3d b1_to_b2 = B2 - B1;
  Eigen::RowVector3d b2_to_b0 = B0 - B2;

  Ray ra0 (A0, a0_to_a1);
  Ray ra1 (A1, a1_to_a2);
  Ray ra2 (A2, a2_to_a0);

  Ray rb0 (B0, b0_to_b1);
  Ray rb1 (B1, b1_to_b2);
  Ray rb2 (B2, b2_to_b0);

  double t;
  bool ra0_hit = ray_intersect_triangle(ra0, B0, B1, B2, 0, 1, t);
  bool ra1_hit = ray_intersect_triangle(ra1, B0, B1, B2, 0, 1, t);
  bool ra2_hit = ray_intersect_triangle(ra2, B0, B1, B2, 0, 1, t);

  bool rb0_hit = ray_intersect_triangle(rb0, A0, A1, A2, 0, 1, t);
  bool rb1_hit = ray_intersect_triangle(rb1, A0, A1, A2, 0, 1, t);
  bool rb2_hit = ray_intersect_triangle(rb2, A0, A1, A2, 0, 1, t);

  return ((ra0_hit)||(ra1_hit)||(ra2_hit)||(rb0_hit)||(rb1_hit)||(rb2_hit));
}
