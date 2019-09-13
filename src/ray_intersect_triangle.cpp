#include "ray_intersect_triangle.h"
#include <vector>
#include <Eigen/Geometry>
#include <limits>

bool ray_intersect_triangle(
  const Ray & ray,
  const Eigen::RowVector3d & A,
  const Eigen::RowVector3d & B,
  const Eigen::RowVector3d & C,
  const double min_t,
  const double max_t,
  double & t)
{
  Eigen::Vector3d e = ray.origin;
  Eigen::Vector3d d = ray.direction;

  Eigen::Vector3d a = (Eigen::Vector3d)A;
  Eigen::Vector3d b = (Eigen::Vector3d)B;
  Eigen::Vector3d c = (Eigen::Vector3d)C;

  Eigen::Vector3d e_to_A = a - e;
  Eigen::Vector3d e_to_B = b - e;
  Eigen::Vector3d e_to_C = c - e;

  double aa = e_to_A.x();
  double dd = e_to_B.x();
  double gg = e_to_C.x();
  double jj = d.x();

  double bb = e_to_A.y();
  double ee = e_to_B.y();
  double hh = e_to_C.y();
  double kk = d.y();

  double cc = e_to_A.z();
  double ff = e_to_B.z();
  double ii = e_to_C.z();
  double ll = d.z();

  int num =3;
  std::vector<double> x(num);

  double M = aa*(ee*ii-hh*ff)+bb*(gg*ff-dd*ii)+cc*(dd*hh-ee*gg);
  x[0] = (jj*(ee*ii-hh*ff)+kk*(gg*ff-dd*ii)+ll*(dd*hh-ee*gg))/M;
  x[1] = (ii*(aa*kk-jj*bb)+hh*(jj*cc-aa*ll)+gg*(bb*ll-kk*cc))/M;
  x[2] = (-1)*(ff*(aa*kk-jj*bb)+ee*(jj*cc-aa*ll)+dd*(bb*ll-kk*cc))/M;

  double vector1[3];
  double vector2[3];

  vector1[0] = b.x() - a.x();
  vector1[1] = b.y() - a.y();
  vector1[2] = b.z() - a.z();

  vector2[0] = c.x() - a.x();
  vector2[1] = c.y() - a.y();
  vector2[2] = c.z() - a.z();

  Eigen::Vector3d normal;
  normal.x()= (vector1[1]*vector2[2])-(vector1[2]*vector2[1]);
  normal.y() = (vector1[2]*vector2[0])-(vector1[0]*vector2[2]);
  normal.z() = (vector1[0]*vector2[1])-(vector1[1]*vector2[0]);

  Eigen::Vector3d n;
  for (int i=0; i<num; i++) {
    if (x[i] < 0) {
      return false;
    }
  }

  if (normal.dot(d) < 0) {
    n = normal/normal.norm();
  }
  else {
    n = -normal/normal.norm();
  }

  double cos_theta = abs(d.dot(n)/(d.norm()*n.norm()));
  double distance = abs(n.dot(e_to_A));
  t = (distance/cos_theta)/d.norm();

  if (t < min_t) {
    return false;
  }
  if (t > max_t) {
    return false;
  }
  if (t == std::numeric_limits<double>::infinity()) {
    return false;
  }
  if (isnan(t)) {
    return false;
  }

  return true;
}

