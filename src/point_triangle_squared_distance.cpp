#include "point_triangle_squared_distance.h"
#include "insert_triangle_into_box.h"
#include <limits>

Eigen::Vector3d cross_product(Eigen::RowVector3d v1, Eigen::RowVector3d v2)
{
  Eigen::Vector3d cross;
  cross.x()= (v1(1)*v2(2))-(v1(2)*v2(1));
  cross.y() = (v1(2)*v2(0))-(v1(0)*v2(2));
  cross.z() = (v1(0)*v2(1))-(v1(1)*v2(0));
  return cross;
}

double point_triangle_squared_distance(
  const Eigen::RowVector3d & query,
  const Eigen::RowVector3d & A,
  const Eigen::RowVector3d & B,
  const Eigen::RowVector3d & C,
  Eigen::RowVector3d & bary)
{
  double eps = std::numeric_limits<double>::epsilon();
  Eigen::RowVector3d q_to_A = A - query;
  Eigen::RowVector3d a = B - C;
  Eigen::RowVector3d b = A - C;
  Eigen::RowVector3d c = A - B;

  Eigen::RowVector3d n = cross_product(a, b);
  Eigen::RowVector3d normal = n/n.norm();
  double h = abs((q_to_A.dot(normal))/(normal.norm()));

  Eigen::RowVector3d p = query + h*normal;
  if ((p - A).dot(normal) != 0) { p = query - h*normal; }

  Eigen::RowVector3d n_a = cross_product(a, normal);
  Eigen::RowVector3d n_b = cross_product(b, normal);
  Eigen::RowVector3d n_c = cross_product(c, normal);

  Eigen::RowVector3d normal_a = n_a/n_a.norm();
  Eigen::RowVector3d normal_b = n_b/n_b.norm();
  Eigen::RowVector3d normal_c = n_c/n_c.norm();

  double h_a = abs(((B - p).dot(n_a))/(n_a.norm()));
  double h_b = abs(((C - p).dot(n_b))/(n_b.norm()));
  double h_c = abs(((A - p).dot(n_c))/(n_c.norm()));

  double cosc = (a.dot(b))/(a.norm()*b.norm());
  double sinc = sqrt(1 - pow(cosc, 2));
  double area = (0.5)*a.norm()*b.norm()*sinc;

  double area_a = (0.5)*(h_a * (a.norm()));
  double area_b = (0.5)*(h_b * (b.norm()));
  double area_c = (0.5)*(h_c * (c.norm()));

  double sum_area = area_a + area_b + area_c;

  if (fabs(area - sum_area) < eps) {
    //in triangle
    bary(0) = area_a;
    bary(1) = area_b;
    bary(2) = area_c;
    return pow(h,2);
  }

  else {
    //out triangle
    Eigen::RowVector3d c1 = p + h_a*normal_a; //lie on BC
    Eigen::RowVector3d c2 = p + h_b*normal_b; //lie on AC
    Eigen::RowVector3d c3 = p + h_c*normal_c; //lie on AB

    if (fabs((c1 - B).dot(normal_a) - 0) > eps) {
      c1 = p - h_a*normal_a;
    }
    if (fabs((c2 - C).dot(normal_b) - 0) > eps) {
      c2 = p - h_b*normal_b;
    }
    if (fabs((c3 - A).dot(normal_c) - 0) > eps) {
      c3 = p - h_c*normal_c;
    }
    Eigen::RowVector3d in_edge= {1,1,1};
    Eigen::RowVector3d height = {h_a,h_b,h_c};
    Eigen::RowVector3d point_to_vertex = {(A - p).norm(),(B - p).norm(),(C - p).norm()};

    if (fabs((c1 - B).norm() + (c1 - C).norm() - (B - C).norm()) > eps) {
      in_edge(0) = 0;
    }
    if (fabs((c2 - C).norm() + (c2 - A).norm() - (A - C).norm()) > eps) {
      in_edge(1) = 0;
    }
    if (fabs((c3 - A).norm() + (c3 - B).norm() - (B - A).norm()) > eps) {
      in_edge(2) = 0;
    }

    for (int i=0; i<3; i++) {
      if (in_edge(i) == 0) {
        height(i) = std::numeric_limits<double>::infinity();
      }
    }
    double min_distance_to_edge = std::numeric_limits<double>::infinity();
    for (int i=0; i<3; i++) {
      if (height(i) < min_distance_to_edge) {
        min_distance_to_edge = height(i);
      }
    }
    double min_distance_to_vertex = std::numeric_limits<double>::infinity();
    for (int i=0; i<3; i++) {
      if (point_to_vertex(i) < min_distance_to_vertex) {
        min_distance_to_vertex = point_to_vertex(i);
      }
    }
    double h2 = std::fmin(min_distance_to_vertex, min_distance_to_edge);

    if (h2 == height(0)) {
      bary(0) = 0;
      bary(1) = ((c1-B).norm())/((C-B).norm());
      bary(2) = ((c1-C).norm())/((C-B).norm());
    }
    if (h2 == height(1)) {
      bary(0) = ((c2-A).norm())/((C-A).norm());
      bary(1) = 0;
      bary(2) = ((c2-C).norm())/((C-A).norm());
    }
    if (h2 == height(2)) {
      bary(0) = ((c3-A).norm())/((B-A).norm());
      bary(1) = ((c3-B).norm())/((B-A).norm());
      bary(2) = 0;
    }
    if (h2 == point_to_vertex(0)) {
      bary(0) = 1;
      bary(1) = 0;
      bary(2) = 0;
    }
    if (h2 == point_to_vertex(1)) {
      bary(0) = 0;
      bary(1) = 1;
      bary(2) = 0;
    }
    if (h2 == point_to_vertex(2)) {
      bary(0) = 0;
      bary(1) = 0;
      bary(2) = 1;
    }
    return pow(h,2) + pow(h2,2);
  }

}
