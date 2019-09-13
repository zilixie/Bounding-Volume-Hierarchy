#include "ray_intersect_box.h"
#include <iostream>
#include "ray_intersect_triangle.h"

bool in_box(
  const Eigen::RowVector3d & p,
  const BoundingBox& box)
{
  double x = p.x();
  double y = p.y();
  double z = p.z();
  Eigen::RowVector3d min_corner = box.min_corner;
  Eigen::RowVector3d max_corner = box.max_corner;
  bool less_than_max = false;
  bool more_than_min = false;

  if ((x >= min_corner.x()) && (y >= min_corner.y()) && (z >= min_corner.z())) {
    more_than_min = true;
  }
  if ((x <= max_corner.x()) && (y <= max_corner.y()) && (z <= max_corner.z())) {
    less_than_max = true;
  }
  if (less_than_max && more_than_min) {
    return true;
  }
  return false;
}

bool ray_intersect_box(
  const Ray & ray,
  const BoundingBox& box,
  const double min_t,
  const double max_t,
  double & t)
{
  Eigen::RowVector3d e = (Eigen::RowVector3d)ray.origin;
  Eigen::RowVector3d d = (Eigen::RowVector3d)ray.direction;

  if (in_box(e, box)){
    t = min_t;
    return true;
  }
  if (in_box(e + min_t*d, box)){
    t = min_t;
    return true;
  }

  t = std::numeric_limits<double>::infinity();
  double another_t;
  Eigen::RowVector3d corner1 = box.max_corner;
  Eigen::RowVector3d corner7 = box.min_corner;
  Eigen::RowVector3d corner2 = {corner7.x(), corner1.y(), corner1.z()};
  Eigen::RowVector3d corner3 = {corner7.x(), corner7.y(), corner1.z()};
  Eigen::RowVector3d corner4 = {corner1.x(), corner7.y(), corner1.z()};
  Eigen::RowVector3d corner5 = {corner1.x(), corner1.y(), corner7.z()};
  Eigen::RowVector3d corner6 = {corner7.x(), corner1.y(), corner7.z()};
  Eigen::RowVector3d corner8 = {corner1.x(), corner7.y(), corner7.z()};

  bool intercept = false;
  if (ray_intersect_triangle(ray, corner1, corner2, corner3, min_t, max_t, another_t)) {
    if (another_t < t) {t = another_t;}
    intercept = true;
  }
  if (ray_intersect_triangle(ray, corner1, corner2, corner4, min_t, max_t, another_t)) {
    if (another_t < t) {t = another_t;}
    intercept = true;
  }
  if (ray_intersect_triangle(ray, corner7, corner3, corner4, min_t, max_t, another_t)) {
    if (another_t < t) {t = another_t;}
    intercept = true;
  }
  if (ray_intersect_triangle(ray, corner7, corner8, corner4, min_t, max_t, another_t)) {
    if (another_t < t) {t = another_t;}
    intercept = true;
  }
  if (ray_intersect_triangle(ray, corner6, corner2, corner3, min_t, max_t, another_t)) {
    if (another_t < t) {t = another_t;}
    intercept = true;
  }
  if (ray_intersect_triangle(ray, corner6, corner3, corner7, min_t, max_t, another_t)) {
    if (another_t < t) {t = another_t;}
    intercept = true;
  }
  // ------------------------------ half of box ---------------------------------------//

  if (ray_intersect_triangle(ray, corner4, corner1, corner5, min_t, max_t, another_t)) {
    if (another_t < t) {t = another_t;}
    intercept = true;
  }
  if (ray_intersect_triangle(ray, corner4, corner8, corner5, min_t, max_t, another_t)) {
    if (another_t < t) {t = another_t;}
    intercept = true;
  }
  if (ray_intersect_triangle(ray, corner5, corner2, corner1, min_t, max_t, another_t)) {
    if (another_t < t) {t = another_t;}
    intercept = true;
  }
  if (ray_intersect_triangle(ray, corner5, corner6, corner1, min_t, max_t, another_t)) {
    if (another_t < t) {t = another_t;}
    intercept = true;
  }
  if (ray_intersect_triangle(ray, corner8, corner7, corner6, min_t, max_t, another_t)) {
    if (another_t < t) {t = another_t;}
    intercept = true;
  }
  if (ray_intersect_triangle(ray, corner8, corner5, corner6, min_t, max_t, another_t)) {
    if (another_t < t) {t = another_t;}
    intercept = true;
  }
  if (intercept) {
    return true;
  }

  return false;
}
