#include "point_AABBTree_squared_distance.h"
#include <queue> // std::priority_queue
#include <vector>
#include <CloudPoint.h>


class AABBTree_and_query {
public:
  Eigen::RowVector3d query;
  std::shared_ptr<Object> root;
  double distance;
  AABBTree_and_query(Eigen::RowVector3d q,
                     std::shared_ptr<Object> r) {
    query = q;
    root = r;
    distance = point_box_squared_distance(query, root->box);
  }
};

struct more
{
  bool operator()(const AABBTree_and_query& a, const AABBTree_and_query& b) const
  {
    return a.distance > b.distance;
  }
};


bool point_AABBTree_squared_distance(
        const Eigen::RowVector3d & query,
        const std::shared_ptr<AABBTree> & root,
        const double min_sqrd,
        const double max_sqrd,
        double & sqrd,
        Eigen::RowVector3d & projection,
        std::shared_ptr<Object> & descendant)
{
  std::priority_queue<AABBTree_and_query, std::vector<AABBTree_and_query>, more> tree;

  AABBTree_and_query alpha (query, root);
  alpha.query = query;
  alpha.root = root;
  tree.push(alpha);
  bool valid = false;
  double this_sqrd = std::numeric_limits<double>::infinity();
  Eigen::RowVector3d this_projection;

  while (!tree.empty()) {
    AABBTree_and_query current_node = tree.top();
    tree.pop();
    if (current_node.distance <= this_sqrd) {
      auto aabb_cast_ptr = std::dynamic_pointer_cast<AABBTree>(current_node.root);

      if (aabb_cast_ptr != nullptr) {
        AABBTree_and_query left_tree(query, aabb_cast_ptr->left);
        AABBTree_and_query right_tree(query, aabb_cast_ptr->right);
        tree.push(left_tree);
        tree.push(right_tree);
      }
      else {
        auto cp_cast_ptr = std::dynamic_pointer_cast<CloudPoint>(current_node.root);
        if (cp_cast_ptr != nullptr) {
          double pl_sqrd = 0;
          Eigen::RowVector3d pl_projection;
          std::shared_ptr<Object> this_descendant;
          valid = current_node.root->point_squared_distance(current_node.query, min_sqrd,
                  max_sqrd, pl_sqrd, pl_projection, this_descendant);

          if (pl_sqrd < this_sqrd) {
            this_sqrd = pl_sqrd;
            this_projection = pl_projection;
            descendant = current_node.root;
          }
        }
      }
    }
  }
  sqrd = this_sqrd;
  projection = this_projection;
  return valid;
}
