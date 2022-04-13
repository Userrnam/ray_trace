#pragma once

#include <vector>

#include "math.hpp"

struct Material {
	vec3 color;
	vec3 emissive;
	float specular; // 1 - very reflective, 0 - not reflective
};

struct Plane {
	vec3 normal;
	float d;
	int mat_index;
};

struct Sphere {
	vec3 pos;
	float r;
	int mat_index;
};

struct World {
	std::vector<Material> materials;
	std::vector<Plane> planes;
	std::vector<Sphere> spheres;

	std::vector<vec3> triangle_vertices;
	std::vector<int> triangle_indices;
	std::vector<int> triangle_materials;
};
