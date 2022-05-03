#pragma once

#include <vector>
#include <assert.h>

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
	std::unordered_map<std::string, int> material_names;
	
	std::vector<Plane> planes;
	std::vector<Sphere> spheres;

	Obj_File obj;
	std::vector<int> mesh_indices;

	void add_material(const std::string& name, const Material& mat) {
		assert(material_names.find(name) == material_names.end());
		material_names[name] = materials.size();
		materials.push_back(mat);
	}

	void add_obj(const std::string& name, const std::string mat_name) {
		auto it = material_names.find(mat_name);
		assert(it != material_names.end());
		int material_index = it->second;
		mesh_indices.push_back(obj.mesh_index[name]);
		obj.meshes[obj.mesh_index[name]].material_index = material_index;
	}
};
