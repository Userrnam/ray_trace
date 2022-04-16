#include <stb_image.h>
#include <vector>
#include <stdlib.h>
#include <limits.h>
#include <iostream>
#include <SFML/Graphics.hpp>

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
		{ 2, 2, 0 },
		{ 0, 4, 0 },
		{ 0, 2, 2 },
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
	vec3 center;
	vec3 vertical;
	vec3 horizontal;

	void create(float width, float height, vec3 _pos, vec3 look_at, float lense_distance) {
		pos = _pos;
		vec3 dir = norm(pos - look_at);
		vec3 _up = { 0, 0, 1 };
		vec3 right = cross(_up, dir);
		vec3 up = cross(dir, right);

		center = pos - lense_distance * dir;
		vertical   = 0.5 * up;
		horizontal = 0.5 * (width / height) * right;
	}

	Ray get_ray(float u, float v) {
		Ray rey;
		rey.origin = pos;
		rey.dir = norm(center + u * horizontal + v * vertical - pos);
		return rey;
	}
};

void render_pass(std::vector<vec3>& img, Camera& camera, World *world, s32 image_width, s32 image_height, int prev) {
	float max_color = {};
    for (int32_t y = 0; y < image_height; ++y) {
        float v = ((float) y / (float) image_height) * -2.0f + 1.0f;       // [-1; 1]
        for (int32_t x = 0; x < image_width; ++x) {
            float u = (((float) x / (float) image_width) * 2.0f - 1.0f);   // [-1; 1]

			Ray ray = camera.get_ray(u, v);

			vec3 color = ray_color(world, ray, 50, 3, prev, img[y * image_width + x]);

			img[y * image_width + x] = color;
        }
    }
}

int main() {
	World world;
	setup_world(&world);

	srand(time(NULL));
	seed(rand());

	s32 image_width  = 720;
	s32 image_height = 480;

	std::vector<vec3> img;
	img.resize(image_width * image_height, {});

	Camera camera;
	camera.create(image_width, image_height, { 0, 10, 1}, { 3, 0, 0 }, 1);

	render_pass(img, camera, &world, image_width, image_height, 0);

	std::vector<Color> buffer;
	buffer.resize(image_width * image_height);
	for (int i = 0; i < img.size(); ++i) {
		buffer[i] = vec_to_color(clamp(img[i], vec3(0,0,0), vec3(1,1,1)));
	}

	stbi_write_png("test.png", image_width, image_height, sizeof(Color), buffer.data(), image_width * sizeof(Color));

	return 0;
}
