#include "bvh.h"

#include "CGL/CGL.h"
#include "scene/bbox.h"
#include "triangle.h"
#include "vector3D.h"

#include <cmath>
#include <cstddef>
#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives, size_t max_leaf_size) {

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

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start, std::vector<Primitive *>::iterator end, size_t max_leaf_size) {

	// TODO (Part 2.1):
	// Construct a BVH from the given vector of primitives and maximum leaf
	// size configuration. The starter code build a BVH aggregate with a
	// single leaf node (which is also the root) that encloses all the
	// primitives.

	BBox bbox;

	for (auto p = start; p != end; p++) {
		BBox bb = (*p)->get_bbox();
		bbox.expand(bb);
	}

	BVHNode *node = new BVHNode(bbox);

	// node->start = start;
	// node->end = end;

	// return node;

	// Base case: if the # of primitives dips below the max_leaf_size, make it a leaf node
	if (end - start <= max_leaf_size) {
		node->l = nullptr;
		node->r = nullptr;
		node->start = start;
		node->end = end;
		return node;
	}
	
	// Find the mean centroid
	Vector3D meanCentroid = Vector3D();
	for (auto p = start; p != end; p++) {
		meanCentroid += (*p)->get_bbox().centroid();
	}
	meanCentroid /= end - start;
	
	int split_axis = 0;
	float min_heuristic = INFINITY;
	std::vector<Primitive*> left;
	std::vector<Primitive*> right;
	// Iterate through axes to see which one is best
	for (int axis = 0; axis < 3; axis++) {
		std::vector<Primitive*> left_primitives;
		BBox left_bbox;
		std::vector<Primitive*> right_primitives;
		BBox right_bbox;

		// Get left and right primitives and bounding boxes
		for (auto p = start; p != end; p++) {
			if ((*p)->get_bbox().centroid()[axis] <= meanCentroid[axis]) {
				left_primitives.push_back(*p);
				left_bbox.expand((*p)->get_bbox());
			} else {
				right_primitives.push_back(*p);
				right_bbox.expand((*p)->get_bbox());
			}
		}

		// Get axis that yields the smallest heuristic
		float heuristic = left_bbox.surface_area() * left_primitives.size() + right_bbox.surface_area() * right_primitives.size();
		if (heuristic <= min_heuristic) {
			min_heuristic = heuristic;
			split_axis = axis;
			left = left_primitives;
			right = right_primitives;
		}
	}

    // Handle degenerate split (when all elements are on the same line so split to one side)
    if (left.empty() || right.empty()) {
        size_t mid = (end - start) / 2;
        left.assign(start, start + mid);
        right.assign(start + mid, end);
    }

	int index = 0;
	for (auto p = start; p != end; p++) {
		if (index < left.size()) {
			*p = left[index];
		} else if (index - left.size() < right.size()) {
			*p = right[index - left.size()];
		}
		index++;
	}

	node->l = construct_bvh(start, start + left.size(), max_leaf_size);
	node->r = construct_bvh(start + left.size(), end, max_leaf_size);

	return node;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
	// TODO (Part 2.3):
	// Fill in the intersect function.
	// Take note that this function has a short-circuit that the
	// Intersection version cannot, since it returns as soon as it finds
	// a hit, it doesn't actually have to find the closest hit.

    // prev code
    // for (auto p : primitives) {
    // total_isects++;
    // if (p->has_intersection(ray))
    //     return true;
    // }
    // return false;

	// This can happen when one bounding box has no primitives
	if (node == nullptr) return false;

	// If the ray doesn't intersect the bounding box, return false
	if (!node->bb.intersect(ray, ray.min_t, ray.max_t)) {
		return false;
	}

	// For leaf nodes, test all primitives intersections
	if (node->isLeaf()) {
		for (auto p = node->start; p != node->end; p++) {
			if ((*p)->has_intersection(ray)) {
				return true;
			}
		}
	}

	// Check leaf nodes' intersections, we can have early returns
	return has_intersection(ray, node->l) || has_intersection(ray, node->r);
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
	// TODO (Part 2.3):
	// Fill in the intersect function.

    // prev code
    // bool hit = false;
    // for (auto p : primitives) {
    //     total_isects++;
    //     hit = p->intersect(ray, i) || hit;
    // }
    // return hit;

	// This can happen when one bounding box has no primitives
	if (node == nullptr) return false;

	// Checking if ray even goes into bounding box
	if (!has_intersection(ray, node)) return false;

	// for leaf nodes, we still have to check if ray hits any primitives
	if (node->isLeaf()) {
		bool intersect = false;
		for (auto p = node->start; p != node->end; p++) {
			intersect = (*p)->intersect(ray, i) || intersect;
		}
		return intersect;
	}

	// Check leaf nodes' intersections, but we need to run as separate lines to ensure it all runs
	bool hit_left = intersect(ray, i, node->l); 
	bool hit_right = intersect(ray, i, node->r);
	return hit_left || hit_right;
}

} // namespace SceneObjects
} // namespace CGL
