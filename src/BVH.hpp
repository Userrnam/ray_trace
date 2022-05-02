#pragma once

#include "Math.hpp"

#include <vector>


struct Bounding_Box {
	vec3 points[2];

	bool intersect(const Ray& ray) const;
};

struct BVH_Node {
    Bounding_Box bounding_box;
    int left = -1;
    int right = -1;

    std::vector<int> triangle_indices;

    // returns vector of triangles to check.
    bool intersect(const std::vector<BVH_Node>& nodes, Ray ray, std::vector<int>& triangle_indices) const;
};

class BVH {
    std::vector<BVH_Node> _nodes;
    int _root;

    BVH_Node build_recursive(struct Obj_File* obj_file, struct Mesh* mesh, int split_axes, const std::vector<int>& triangle_indices);
public:
    void build(struct Mesh* mesh, struct Obj_File* obj_file);

    // returns vector of triangles to check.
    bool intersect(Ray ray, std::vector<int>& triangle_indices) const;
};

