#pragma once

#include <vector>

#include "Math.hpp"
#include "Obj_Loader.hpp"


struct Material {
	vec3 color = {};
	vec3 emissive = {};
	float specular = 0; // 1 - very reflective, 0 - not reflective
	float refractiveness = 0;
	float n = 1; // refraction coefficient
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

	Obj_File obj;
	std::vector<int> mesh_indices;
	std::vector<int> mesh_materials;

	void add_obj(const std::string& name, int material_index) {
		mesh_indices.push_back(obj.mesh_index[name]);
		mesh_materials.push_back(material_index);
	}
};
