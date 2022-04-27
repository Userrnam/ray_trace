#include <vector>
#include <stdlib.h>
#include <limits.h>
#include <iostream>

#include "Application.hpp"
#include "Job_System.hpp"

#include "CPURenderer.hpp"

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
		.specular = 1.0,
		.refractiveness = 1.0,
		.n = 1.7
	});

	world.materials.push_back({
		// Light
		.color = { 1.0f, 1.0f, 1.0f },
		.emissive = { 3.0f, 3.0f, 3.0f }
	});

	for (const auto& mesh : world.obj.meshes) {
		std::cout << mesh.name << std::endl;
	}

	world.add_obj("Floor", 1);
	world.add_obj("Ceiling", 1);
	world.add_obj("Light", 3);
	world.add_obj("LeftWall", 1);
	world.add_obj("RightWall", 1);
	world.add_obj("BackWall", 1);
	world.add_obj("Suzanne", 2);

	Application app;
	if (!app.init(800, 600, new CPURenderer(6))) {
		return -1;
	}

	app.set_world(&world);
	app.run();

	return 0;
}
