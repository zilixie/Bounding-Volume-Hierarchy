#include "AABBTree.h"
#include "insert_box_into_box.h"
#include <typeinfo>
#include "MeshTriangle.h"

AABBTree::AABBTree(
  const std::vector<std::shared_ptr<Object> > & objects,
  int a_depth): 
  depth(std::move(a_depth)), 
  num_leaves(objects.size())
{
  for (int i=0; i<objects.size(); i++) {
    BoundingBox object_bounding_box = objects[i]->box;
    insert_box_into_box(object_bounding_box, this->box);
  }

  double x_axis = this->box.max_corner.x() - this->box.min_corner.x();
  double y_axis = this->box.max_corner.y() - this->box.min_corner.y();
  double z_axis = this->box.max_corner.z() - this->box.min_corner.z();

  double tmp = ((x_axis < y_axis) ? y_axis:x_axis);
  double max_axis = ((tmp < z_axis) ? z_axis:tmp);

  std::vector<std::shared_ptr<Object> > left_objects;
  std::vector<std::shared_ptr<Object> > right_objects;

  int switcher = 1;
  int index = 0;
  if (max_axis == x_axis) { index = 0; }
  else if (max_axis == y_axis) { index = 1; }
  else if (max_axis == z_axis) { index = 2; }

  double mid_point = (this->box.max_corner(index) + this->box.min_corner(index))/2;
  for (int j=0; j<objects.size(); j++) {
    if (objects[j]->box.center()(index) < mid_point) {
      left_objects.push_back(objects[j]);
      switcher = 0;
    }
    else if (objects[j]->box.center()(index) > mid_point) {
      right_objects.push_back(objects[j]);
      switcher = 1;
    }
  }
  for (int k=0; k<objects.size(); k++) {
    if (objects[k]->box.center()(index) == mid_point) {
      if (switcher) {
        left_objects.push_back(objects[k]);
        switcher = 0;
      }
      else {
        right_objects.push_back(objects[k]);
        switcher = 1;
      }
    }
  }

  if (left_objects.size() > 1) {
    auto left_tree = AABBTree(left_objects, depth + 1);
    this->left = std::make_shared<AABBTree>(left_tree);
  }
  else {
    this->left = left_objects[0];
  }
  if (right_objects.size() > 1) {
    auto right_tree = AABBTree(right_objects, depth + 1);
    this->right = std::make_shared<AABBTree>(right_tree);
  }
  else {
    this->right = right_objects[0];
  }
}

bool AABBTree::ray_intersect(
        const Ray& ray,
        const double min_t,
        const double max_t,
        double & t,
        std::shared_ptr<Object> & descendant) const
{
  bool left_intersect = false;
  bool right_intersect = false;
  double this_t = std::numeric_limits<double>::infinity();
  double another_t = std::numeric_limits<double>::infinity();
  if (ray_intersect_box(ray, this->box, min_t, max_t, another_t)) {
    double left_t;
    double right_t;
    std::shared_ptr<Object> left_descendant;
    std::shared_ptr<Object> right_descendant;

    if (ray_intersect_box(ray, this->left->box, min_t, max_t, left_t)) {
      left_intersect = this->left->ray_intersect(ray, min_t, max_t, left_t, left_descendant);
      if (left_intersect) {
        this_t = left_t;
        if (std::dynamic_pointer_cast<MeshTriangle>(this->left)) {
          left_descendant = this->left;
        }
      }
    }
    if (ray_intersect_box(ray, this->right->box, min_t, max_t, right_t)) {
      right_intersect = this->right->ray_intersect(ray, min_t, max_t, right_t, right_descendant);
      if (right_intersect) {
        this_t = right_t;
        if (std::dynamic_pointer_cast<MeshTriangle>(this->right)) {
          right_descendant = this->right;
        }
      }
    }
    if (left_intersect && right_intersect) {
      this_t = std::fmin(left_t, right_t);
      if (this_t == left_t) {
        descendant = left_descendant;
      }
      if (this_t == right_t){
        descendant = right_descendant;
      }
    }
    else if (left_intersect) {
      descendant = left_descendant;
    }
    else if (right_intersect) {
      descendant = right_descendant;
    }
  }
  t = this_t;
  return (left_intersect || right_intersect);
}
