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
	world.obj.load("cornell_box_new.obj");

	for (auto name : world.obj.mesh_index) {
		std::cout << name.first << std::endl;
	}

	world.add_material("sky", {
		.color    = { 0.3f, 0.4f, 0.8f },
		});

	world.add_material("red", {
		.color = { 0.9f, 0.0f, 0.0f }
		});

	world.add_material("blue", {
		.color = { 0.1f, 0.1f, 0.7f },
		.specular = 1
		});

	world.add_material("light_grey", {
		.color = { 0.8f, 0.8f, 0.8f }
		});

	world.add_material("green", {
		.color = { 0.1f, 0.9f, 0.1f }
		});

	world.add_material("glass", {
		.color = { 1.0f, 1.0f, 1.0f },
		.specular = 1.0,
		.refractiveness = 1.0,
		.n = 1.7
		});

	world.add_material("purple", {
		.color = { 0.39f, 0.26f, 0.84f },
		});

	world.add_material("mirror", {
		.color = { 1.0f, 1.0f, 1.0f },
		.specular = 1
		});

	world.add_material("yellow", {
		.color = { 1.0f, 1.0f, 0.0f },
		.specular = 0.2
		});

	world.add_material("white_light", {
		.color = { 8.0f, 8.0f, 8.0f },
		.emissive = true
		});

	world.add_obj("Cube", "blue");
	world.add_obj("Ceiling", "light_grey");
	world.add_obj("Floor", "purple");
	world.add_obj("RightWall", "red");
	world.add_obj("LeftWall", "green");
	world.add_obj("BackWall", "light_grey");
	world.add_obj("Suzanne", "yellow");
	world.add_obj("Light", "white_light");
	world.add_obj("FrontWall", "light_grey");

	world.spheres = {
		{
			.pos = { -3, 3, 3.5 },
			.r = 1,
			.mat_index = world.material_names["glass"]
		},
		{
			.pos = { -6, -6, 6 },
			.r = 4,
			.mat_index = world.material_names["mirror"]
		},
	};

	Application app;
	//if (!app.init(800, 600, new CPURenderer(10))) {
	if (!app.init(800, 600, new GPURenderer)) {
		return -1;
	}

	app.set_world(&world);
	app.run();

	return 0;
}
