#include "point_box_squared_distance.h"

Eigen::Vector3d cross_product(Eigen::RowVector3d v1, Eigen::RowVector3d v2)
{
  Eigen::Vector3d cross;
  cross.x()= (v1(1)*v2(2))-(v1(2)*v2(1));
  cross.y() = (v1(2)*v2(0))-(v1(0)*v2(2));
  cross.z() = (v1(0)*v2(1))-(v1(1)*v2(0));
  return cross;
}

double point_box_squared_distance(
  const Eigen::RowVector3d & query,
  const BoundingBox & box)
{
  Eigen::RowVector3d corner1 = box.max_corner;
  Eigen::RowVector3d corner7 = box.min_corner;
  Eigen::RowVector3d corner2 = {corner7.x(), corner1.y(), corner1.z()};
  Eigen::RowVector3d corner3 = {corner7.x(), corner7.y(), corner1.z()};
  Eigen::RowVector3d corner4 = {corner1.x(), corner7.y(), corner1.z()};
  Eigen::RowVector3d corner5 = {corner1.x(), corner1.y(), corner7.z()};
  Eigen::RowVector3d corner6 = {corner7.x(), corner1.y(), corner7.z()};
  Eigen::RowVector3d corner8 = {corner1.x(), corner7.y(), corner7.z()};

  Eigen::RowVector3d normal1 = cross_product((corner2-corner1), (corner4-corner1));
  Eigen::RowVector3d normal2 = cross_product((corner4-corner8), (corner7-corner8));
  Eigen::RowVector3d normal3 = cross_product((corner4-corner1), (corner5-corner1));
  Eigen::RowVector3d normal4 = cross_product((corner5-corner1), (corner2-corner1));
  Eigen::RowVector3d normal5 = cross_product((corner6-corner2), (corner3-corner2));
  Eigen::RowVector3d normal6 = cross_product((corner8-corner5), (corner6-corner5));

  double d1 = abs(((corner1-query).dot(normal1))/(normal1.norm()));
  double d2 = abs(((corner8-query).dot(normal2))/(normal2.norm()));
  double d3 = abs(((corner1-query).dot(normal3))/(normal3.norm()));
  double d4 = abs(((corner1-query).dot(normal4))/(normal4.norm()));
  double d5 = abs(((corner2-query).dot(normal5))/(normal5.norm()));
  double d6 = abs(((corner5-query).dot(normal6))/(normal6.norm()));

  if (corner1 == corner2) {
    d1 = abs(corner1.z() - query.z());
    d6 = abs(corner7.z() - query.z());
    d2 = abs(corner7.y() - query.y());
    d4 = abs(corner1.y() - query.y());
  }
  if (corner1 == corner4) {
    d1 = abs(corner1.z() - query.z());
    d6 = abs(corner7.z() - query.z());
    d3 = abs(corner1.x() - query.x());
    d5 = abs(corner7.x() - query.x());
  }
  if (corner1 == corner5) {
    d2 = abs(corner7.y() - query.y());
    d4 = abs(corner1.y() - query.y());
    d3 = abs(corner1.x() - query.x());
    d5 = abs(corner7.x() - query.x());
  }

  Eigen::RowVector3d closest_plane = {std::fmin(d3,d5), std::fmin(d2,d4), std::fmin(d1,d6)};
  double sqrD = pow(closest_plane(0),2) + pow(closest_plane(1),2) + pow(closest_plane(2),2);

  for (int i=0;i<3; i++) {
    if ((box.min_corner(i) < query(i))&& (query(i) < box.max_corner(i))) {
      sqrD = sqrD - pow(closest_plane(i),2);
    }
  }

  return sqrD;
}
