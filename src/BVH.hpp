#pragma once

#include "Math.hpp"

#include <vector>


struct Bounding_Box {
	vec3 points[2];

	float intersect(const Ray& ray) const;
};

struct BVH_Node {
    Bounding_Box bounding_box;
    int left = -1;
    int right = -1;

    int first_triangle = 0;
    int triangle_count = 0;

    // returns vector of triangles to check.
    bool intersect(const struct Obj_File* obj_file, int vi_start, Ray ray, int* triangle_indices, float *t) const;
};

class BVH {
    int _first = 0; // point to nodes in obj file

    BVH_Node build_recursive(struct Obj_File* obj_file, struct Mesh* mesh, const std::vector<int>& triangle_indices);
public:
    void build(struct Mesh* mesh, struct Obj_File* obj_file);

    // returns vector of triangles to check.
    bool intersect(const struct Obj_File* obj_file, int vi_start, Ray ray, int* triangle_index, float *t) const;
};

