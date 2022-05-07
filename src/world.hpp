#pragma once

#include <vector>
#include <assert.h>

#include "Math.hpp"
#include "Obj_Loader.hpp"


struct Material {
	vec3 color = {};
	bool emissive = false;
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
		if (material_names.find(name) != material_names.end()) {
			std::cout << "Error: material " << name << " already exists.\n";
			return;
		}
		material_names[name] = materials.size();
		materials.push_back(mat);
	}

	void add_obj(const std::string& name, const std::string mat_name) {
		if (material_names.find(mat_name) == material_names.end()) {
			std::cout << "Error: material " << mat_name << " does not exists.\n";
			return;
		}
		if (obj.mesh_index.find(name) == obj.mesh_index.end()) {
			std::cout << "Error: Object " << name << " does not exists.\n";
			return;
		}
		int material_index = material_names[mat_name];
		mesh_indices.push_back(obj.mesh_index[name]);
		obj.meshes[obj.mesh_index[name]].material_index = material_index;
	}
};
