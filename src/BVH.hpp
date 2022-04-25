#pragma once

#include "Math.hpp"

#include <vector>


struct Bounding_Box {
	vec3 points[2];

	bool intersect(const Ray& ray) const;
};

struct BVH_Node {
    Bounding_Box bounding_box;
    BVH_Node* left  = nullptr;
    BVH_Node* right = nullptr;
    // TODO: store triangle indices
    // if it's leaf
    std::vector<int> vertex_indices;
    std::vector<int> normal_indices;

    // returns vector of triangles to check.
    bool intersect(Ray ray, std::vector<int>& vertex_indices, std::vector<int>& normal_indices) const;
};

class BVH {
    BVH_Node* _root;

    BVH_Node* build_recursive(struct Obj_File* obj_file, int split_axes, const std::vector<int>& vertex_indices, const std::vector<int>& normal_indices);
    void destroy_recursive(BVH_Node *node);
public:
    void build(struct Mesh* mesh, struct Obj_File* obj_file);
    void destroy();

    // returns vector of triangles to check.
    bool intersect(Ray ray, std::vector<int>& vertex_indices, std::vector<int>& normal_indices) const;
};

