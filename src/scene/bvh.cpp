#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}



BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.


  BBox bbox;

  // Compute the bounding box of a list of primitives and initialize a new BVHNode with that bounding box
  int count = 0;
  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
    count += 1;
  }
  BVHNode *node = new BVHNode(bbox);

  // If there are no more than max_leaf_size primitives in the list, 
  // the node we just created is a leaf node and we should update its 
  // start and end iterators appropriately.
  if (count <= max_leaf_size) {
      node->start = start;
      node->end = end;
      return node;
  }

  // Calculate the longest dimension; that is the axis to split
  int dim;
  Vector3D e = bbox.extent;
  if (e.x > e.y && e.x > e.z) {
      dim = 0;
  } else if (e.y > e.x && e.y > e.z) {
      dim = 1;
  } else {
      dim = 2;
  }

  // Compute the split point along this axis
  Vector3D split = Vector3D(0,0,0);
  for (auto p = start; p != end; p++) {
      split += (*p)->get_bbox().centroid();
  }
  split /= count;


  // Divide all primitives into a "left" and "right" collection based on the 
  // centroid of their bounding boxes
  // TODO: std::partition might be faster
  vector<Primitive*> left, right;

  for (auto p = start; p != end; p++) {
      Vector3D c = (*p)->get_bbox().centroid();
      double c_value, split_value;
      if (dim == 0) {
          c_value = c.x;
          split_value = split.x;
      }
      else if (dim == 1) {
          c_value = c.y;
          split_value = split.y;
      }
      else {
          c_value = c.z;
          split_value = split.z;
      }
      if (c_value < split_value) {
          left.push_back(*p);
      }
      else {
          right.push_back(*p);
      }
  }

  // If all primitives lie on one side of the split point, we move the split point 
  // so that both sides will have at least one item
  // TODO: Need to sort left and right
  if (left.empty()) {
      left.push_back(*(right.end()));
      right.pop_back();
  }
  else if (right.empty()) {
      right.push_back(*(left.end()));
      left.pop_back();
  }

  // Set the current node's left and right children by 
  // recursively calling BVHAccel:construct_bvh(...)
  node->l = construct_bvh(left.begin(), left.end(), max_leaf_size);
  node->r = construct_bvh(right.begin(), right.end(), max_leaf_size);

  return node;


}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.



  for (auto p : primitives) {
    total_isects++;
    if (p->has_intersection(ray))
      return true;
  }
  return false;


}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.



  bool hit = false;
  for (auto p : primitives) {
    total_isects++;
    hit = p->intersect(ray, i) || hit;
  }
  return hit;


}

} // namespace SceneObjects
} // namespace CGL
