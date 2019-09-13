#include "box_box_intersect.h"
#include "ray_intersect_box.h"

bool box_box_intersect(
        const BoundingBox & A,
        const BoundingBox & B)
{
  Eigen::RowVector3d a1 = A.max_corner;
  Eigen::RowVector3d a7 = A.min_corner;
  Eigen::RowVector3d a2 = {a7.x(), a1.y(), a1.z()};
  Eigen::RowVector3d a3 = {a7.x(), a7.y(), a1.z()};
  Eigen::RowVector3d a4 = {a1.x(), a7.y(), a1.z()};
  Eigen::RowVector3d a5 = {a1.x(), a1.y(), a7.z()};
  Eigen::RowVector3d a6 = {a7.x(), a1.y(), a7.z()};
  Eigen::RowVector3d a8 = {a1.x(), a7.y(), a7.z()};

  Eigen::RowVector3d b1 = B.max_corner;
  Eigen::RowVector3d b7 = B.min_corner;
  Eigen::RowVector3d b2 = {b7.x(), b1.y(), b1.z()};
  Eigen::RowVector3d b3 = {b7.x(), b7.y(), b1.z()};
  Eigen::RowVector3d b4 = {b1.x(), b7.y(), b1.z()};
  Eigen::RowVector3d b5 = {b1.x(), b1.y(), b7.z()};
  Eigen::RowVector3d b6 = {b7.x(), b1.y(), b7.z()};
  Eigen::RowVector3d b8 = {b1.x(), b7.y(), b7.z()};

  Ray ra1 (a1, a2 - a1);
  Ray ra2 (a2, a3 - a2);
  Ray ra3 (a3, a4 - a3);
  Ray ra4 (a4, a1 - a4);
  Ray ra5 (a5, a6 - a5);
  Ray ra6 (a6, a7 - a6);
  Ray ra7 (a7, a8 - a7);
  Ray ra8 (a8, a5 - a8);
  Ray ra9 (a1, a5 - a1);
  Ray ra10 (a2, a6 - a2);
  Ray ra11 (a3, a7 - a3);
  Ray ra12 (a4, a8 - a4);

  Ray rb1 (b1, b2 - b1);
  Ray rb2 (b2, b3 - b2);
  Ray rb3 (b3, b4 - b3);
  Ray rb4 (b4, b1 - b4);
  Ray rb5 (b5, b6 - b5);
  Ray rb6 (b6, b7 - b6);
  Ray rb7 (b7, b8 - b7);
  Ray rb8 (b8, b5 - b8);
  Ray rb9 (b1, b5 - b1);
  Ray rb10 (b2, b6 - b2);
  Ray rb11 (b3, b7 - b3);
  Ray rb12 (b4, b8 - b4);

  double t;

  bool ra1_hit = ray_intersect_box(ra1, B, 0, 1, t);
  bool ra2_hit = ray_intersect_box(ra2, B, 0, 1, t);
  bool ra3_hit = ray_intersect_box(ra3, B, 0, 1, t);
  bool ra4_hit = ray_intersect_box(ra4, B, 0, 1, t);
  bool ra5_hit = ray_intersect_box(ra5, B, 0, 1, t);
  bool ra6_hit = ray_intersect_box(ra6, B, 0, 1, t);
  bool ra7_hit = ray_intersect_box(ra7, B, 0, 1, t);
  bool ra8_hit = ray_intersect_box(ra8, B, 0, 1, t);
  bool ra9_hit = ray_intersect_box(ra9, B, 0, 1, t);
  bool ra10_hit = ray_intersect_box(ra10, B, 0, 1, t);
  bool ra11_hit = ray_intersect_box(ra11, B, 0, 1, t);
  bool ra12_hit = ray_intersect_box(ra12, B, 0, 1, t);

  bool rb1_hit = ray_intersect_box(rb1, A, 0, 1, t);
  bool rb2_hit = ray_intersect_box(rb2, A, 0, 1, t);
  bool rb3_hit = ray_intersect_box(rb3, A, 0, 1, t);
  bool rb4_hit = ray_intersect_box(rb4, A, 0, 1, t);
  bool rb5_hit = ray_intersect_box(rb5, A, 0, 1, t);
  bool rb6_hit = ray_intersect_box(rb6, A, 0, 1, t);
  bool rb7_hit = ray_intersect_box(rb7, A, 0, 1, t);
  bool rb8_hit = ray_intersect_box(rb8, A, 0, 1, t);
  bool rb9_hit = ray_intersect_box(rb9, A, 0, 1, t);
  bool rb10_hit = ray_intersect_box(rb10, A, 0, 1, t);
  bool rb11_hit = ray_intersect_box(rb11, A, 0, 1, t);
  bool rb12_hit = ray_intersect_box(rb12, A, 0, 1, t);

  bool A_hit = (ra1_hit||ra2_hit||ra3_hit||ra4_hit||ra5_hit||ra6_hit||ra7_hit||ra8_hit||ra9_hit||ra10_hit||ra11_hit||ra12_hit);
  bool B_hit = (rb1_hit||rb2_hit||rb3_hit||rb4_hit||rb5_hit||rb6_hit||rb7_hit||rb8_hit||rb9_hit||rb10_hit||rb11_hit||rb12_hit);

  return (A_hit||B_hit);
}

