#include <vector>
#include <stdlib.h>
#include <limits.h>
#include <iostream>

#include "Application.hpp"
#include "Bounding_Box.hpp"
#include "Job_System.hpp"


int main() {
	srand(time(NULL));
	seed(rand());

	World world;
	world.obj.load("cornell_box.obj");

	world.materials.push_back({
		// sky
		.color    = { 0.3f, 0.4f, 0.8f },
		.emissive = { 0.5f, 0.6f, 0.8f },
		.specular = 0.0f
	});

	world.materials.push_back({
		// Floor
		.color    = { 1.0f, 0.0f, 0.0f },
	});

	world.materials.push_back({
		// Suzanne
		.color    = { 1.0f, 1.0f, 0.0f },
	});

	for (const auto& mesh : world.obj.meshes) {
		std::cout << mesh.name << std::endl;
	}

	world.mesh_indices.push_back(world.obj.mesh_index["Floor"]);
	//world.mesh_indices.push_back(world.obj.mesh_index["Ceiling"]);
	//world.mesh_indices.push_back(world.obj.mesh_index["LeftWall"]);
	//world.mesh_indices.push_back(world.obj.mesh_index["RightWall"]);
	world.mesh_indices.push_back(world.obj.mesh_index["Suzanne"]);

	world.mesh_materials.push_back(1);
	//world.mesh_materials.push_back(1);
	//world.mesh_materials.push_back(1);
	//world.mesh_materials.push_back(1);
	world.mesh_materials.push_back(2);

	Application app;
	if (!app.init(800, 600)) {
		return -1;
	}

	app.set_world(&world);
	app.run();

	return 0;
}
