#include <vector>
#include <stdlib.h>
#include <limits.h>
#include <iostream>

#include "Application.hpp"

void setup_world(World *world) {
	world->materials = {
		Material {
			.color    = { 0.3f, 0.4f, 0.8f },
			.emissive = { 0.5f, 0.6f, 0.8f },
			.specular = 0.0f
		},
		Material {
			.color = { 0.9f, 0.9f, 0.2f },
			.emissive = {},
			.specular = 0.3f
		},
		Material {
			.color = { 0.1f, 0.2f, 0.8f },
			.emissive = {},
			.specular = 0.9f
		},
		Material {
			.color = { 1.0f, 0.0f, 0.0f },
			.emissive = {},
			.specular = 0.0f
		},
		Material {
			.color = { 1.0f, 1.0f, 0.0f },
			.emissive = {},
			.specular = 1.0f
		},
	};

	world->planes = {
		Plane {
			.normal = { 0, 0, 1 },
			.d = 0,
			.mat_index = 1,
		}
	};

	world->spheres = {
		// Sphere {
		// 	.pos = { 5, 2, 2 },
		// 	.r = 2,
		// 	.mat_index = 3
		// },
	// 	Sphere {
	// 		.pos = { 0, 0, 2.1 },
	// 		.r = 2,
	// 		.mat_index = 2
	// 	},
	};

	world->triangle_vertices = {
		{ 2, 2, 0 },
		{ 0, 4, 0 },
		{ 0, 2, 2 },
	};

	world->triangle_indices = { 0, 2, 1 };
	world->triangle_materials = { 4, 4 };
}

int main() {
	srand(time(NULL));
	seed(rand());

	World world;
	setup_world(&world);

	Application app;
	if (!app.init(800, 600)) {
		return -1;
	}

	app.set_world(&world);
	app.run();

	return 0;
}
