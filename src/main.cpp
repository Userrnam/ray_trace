#include <vector>
#include <stdlib.h>
#include <limits.h>
#include <iostream>

#include "Application.hpp"
#include "Job_System.hpp"

#include "CPURenderer.hpp"
#include "GPURenderer.hpp"

int main() {
	srand(time(NULL));
	seed(rand());

	World world;
	world.obj.load("cornell_box.obj");

	for (auto name : world.obj.mesh_index) {
		std::cout << name.first << std::endl;
	}

	world.add_material("sky", {
		.color    = { 0.3f, 0.4f, 0.8f },
		.emissive = { 0.5f, 0.6f, 0.8f },
		.specular = 0.0f
		});

	world.add_material("red", {
		.color = { 1.0f, 0.0f, 0.0f }
		});

	world.add_material("grey", {
		.color = { 0.1f, 0.1f, 0.1f }
		});

	world.add_material("yellow_glass", {
		.color = { 1.0f, 1.0f, 0.0f },
		.specular = 1.0,
		.refractiveness = 1.0,
		.n = 1.7
		});

	world.add_material("white_light", {
		.color = { 1.0f, 1.0f, 1.0f },
		.emissive = { 3.0f, 3.0f, 3.0f }
		});

	world.add_obj("Floor", "grey");
	world.add_obj("Ceiling", "red");
	world.add_obj("Light", "white_light");
	world.add_obj("LeftWall", "red");
	world.add_obj("RightWall", "red");
	world.add_obj("BackWall", "red");
	world.add_obj("Suzanne", "yellow_glass");

	Application app;
	if (!app.init(800, 600, new CPURenderer(6))) {
	//if (!app.init(800, 600, new GPURenderer)) {
		return -1;
	}

	app.set_world(&world);
	app.run();

	return 0;
}
