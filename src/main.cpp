#include <stb_image.h>
#include <vector>
#include <stdlib.h>
#include <limits.h>
#include <iostream>

#include "world.hpp"
#include "ray_trace.hpp"

void setup_world(World *world) {
	world->materials = {
		Material {
			.color    = { 0.3f, 0.4f, 0.8f },
			.emissive = { 0.5f, 0.6f, 0.8f },
			.specular = 0.0f
		},
		Material {
			.color = { 0.5f, 0.2f, 0.2f },
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
			.specular = 0.0f
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
		Sphere {
			.pos = { 0, 0, 2.1 },
			.r = 2,
			.mat_index = 2
		},
		Sphere {
			.pos = { 5, 2, 2.1 },
			.r = 2,
			.mat_index = 3
		},
	};

	world->triangle_vertices = {
		{ 2, 0, 0 },
		{ 0, 2, 0 },
		{ 0, 0, 2 },
	};

	world->triangle_indices = { 0, 1, 2 };
	world->triangle_materials = { 4 };
}

struct Color {
	u8 r;
	u8 g;
	u8 b;
};

Color vec_to_color(vec3 v) {
	return Color {
		u8(255 * v.x),
		u8(255 * v.y),
		u8(255 * v.z),
	};
}

struct Camera {
	vec3 pos;
	vec3 look_at;
	vec3 dir;
	vec3 up;
	vec3 right;

	void update() {
		dir = norm(pos - look_at);
		vec3 _up = { 0, 0, 1 };
		right = cross(_up, dir);
		up = cross(dir, right);
	}
};

int main() {
	World world;
	setup_world(&world);

	seed(rand());

	s32 image_width  = 720;
	s32 image_height = 480;

	Camera camera;
	camera.pos     = { 0, 10, 1 };
	camera.look_at = { 0, 0, 0 };
	camera.update();

	float film_distance = 1.0;
    vec3 film_center = camera.pos - film_distance * camera.dir;

    float film_width  = 1.0f * float(image_width) / float(image_height);
    float film_height = 1.0f;

    float half_film_width  = film_width * 0.5f;
    float half_film_height = film_height * 0.5f;

	std::vector<Color> img;
	img.resize(image_width * image_height);

	float max_color = {};
    for (int32_t y = 0; y < image_height; ++y) {
        float film_y = ((float) y / (float) image_height) * -2.0f + 1.0f;       // [-1; 1]
        for (int32_t x = 0; x < image_width; ++x) {
            float film_x = (((float) x / (float) image_width) * 2.0f - 1.0f);   // [-1; 1]

			vec3 film_position = film_center + film_x * half_film_width * camera.right + film_y * half_film_height * camera.up;

			Ray ray = {};
			ray.origin = camera.pos;
			ray.dir = norm(film_position - camera.pos);

			vec3 color = ray_color(&world, ray, 1000, 5);

			img[y * image_width + x] = vec_to_color(clamp(color, vec3(0,0,0), vec3(1,1,1)));
        }
    }

	stbi_write_png("test.png", image_width, image_height, sizeof(Color), img.data(), image_width * sizeof(Color));

	return 0;
}
