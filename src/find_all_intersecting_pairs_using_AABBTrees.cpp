#include "find_all_intersecting_pairs_using_AABBTrees.h"
#include "box_box_intersect.h"
// Hint: use a list as a queue
#include <list>
#include "MeshTriangle.h"
#include "triangle_triangle_intersection.h"

void find_all_intersecting_pairs_using_AABBTrees(
  const std::shared_ptr<AABBTree> & rootA,
  const std::shared_ptr<AABBTree> & rootB,
  std::vector<std::pair<std::shared_ptr<Object>,std::shared_ptr<Object> > > & 
    leaf_pairs)
{
  std::list<std::pair<std::shared_ptr<Object>,std::shared_ptr<Object> >> q;
  if (box_box_intersect(rootA->box, rootB->box)) {
    std::pair<std::shared_ptr<Object>,std::shared_ptr<Object> > p (rootA, rootB);
    q.push_back(p);
  }

  while (!q.empty()) {
    std::pair<std::shared_ptr<Object>,std::shared_ptr<Object> > current_pair = q.front();
    q.pop_front();

    auto A_leaf_cast_ptr = std::dynamic_pointer_cast<MeshTriangle>(current_pair.first);
    auto B_leaf_cast_ptr = std::dynamic_pointer_cast<MeshTriangle>(current_pair.second);
    if ((A_leaf_cast_ptr!= nullptr) && (B_leaf_cast_ptr!= nullptr)) {
      std::pair<std::shared_ptr<Object>,std::shared_ptr<Object> > p (A_leaf_cast_ptr, B_leaf_cast_ptr);
      leaf_pairs.push_back(p);
    }
    else if ((A_leaf_cast_ptr!= nullptr) && (B_leaf_cast_ptr== nullptr)) {
      auto B_aabb_cast_ptr = std::dynamic_pointer_cast<AABBTree>(current_pair.second);
      if (box_box_intersect(A_leaf_cast_ptr->box, B_aabb_cast_ptr->left->box)) {
        std::pair<std::shared_ptr<Object>,std::shared_ptr<Object> > p (A_leaf_cast_ptr, B_aabb_cast_ptr->left);
        q.push_back(p);
      }
      if (box_box_intersect(A_leaf_cast_ptr->box, B_aabb_cast_ptr->right->box)) {
        std::pair<std::shared_ptr<Object>,std::shared_ptr<Object> > p (A_leaf_cast_ptr, B_aabb_cast_ptr->right);
        q.push_back(p);
      }
    }
    else if ((A_leaf_cast_ptr== nullptr) && (B_leaf_cast_ptr!= nullptr)) {
      auto A_aabb_cast_ptr = std::dynamic_pointer_cast<AABBTree>(current_pair.first);
      if (box_box_intersect(A_aabb_cast_ptr->left->box, B_leaf_cast_ptr->box)) {
        std::pair<std::shared_ptr<Object>,std::shared_ptr<Object> > p (A_aabb_cast_ptr->left, B_leaf_cast_ptr);
        q.push_back(p);
      }
      if (box_box_intersect(A_aabb_cast_ptr->right->box, B_leaf_cast_ptr->box)) {
        std::pair<std::shared_ptr<Object>,std::shared_ptr<Object> > p (A_aabb_cast_ptr->right, B_leaf_cast_ptr);
        q.push_back(p);
      }
    }
    else {
      auto A_aabb_cast_ptr = std::dynamic_pointer_cast<AABBTree>(current_pair.first);
      auto B_aabb_cast_ptr = std::dynamic_pointer_cast<AABBTree>(current_pair.second);
      if (box_box_intersect(A_aabb_cast_ptr->left->box, B_aabb_cast_ptr->left->box)) {
        std::pair<std::shared_ptr<Object>,std::shared_ptr<Object> > p (A_aabb_cast_ptr->left, B_aabb_cast_ptr->left);
        q.push_back(p);
      }
      if (box_box_intersect(A_aabb_cast_ptr->left->box, B_aabb_cast_ptr->right->box)) {
        std::pair<std::shared_ptr<Object>,std::shared_ptr<Object> > p (A_aabb_cast_ptr->left, B_aabb_cast_ptr->right);
        q.push_back(p);
      }
      if (box_box_intersect(A_aabb_cast_ptr->right->box, B_aabb_cast_ptr->left->box)) {
        std::pair<std::shared_ptr<Object>,std::shared_ptr<Object> > p (A_aabb_cast_ptr->right, B_aabb_cast_ptr->left);
        q.push_back(p);
      }
      if (box_box_intersect(A_aabb_cast_ptr->right->box, B_aabb_cast_ptr->right->box)) {
        std::pair<std::shared_ptr<Object>,std::shared_ptr<Object> > p (A_aabb_cast_ptr->right, B_aabb_cast_ptr->right);
        q.push_back(p);
      }
    }
  }
}
