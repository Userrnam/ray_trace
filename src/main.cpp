#include "math.hpp"

#include <stb_image.h>
#include <vector>
#include <limits.h>

struct Material {
	vec3 color;
	vec3 emissive;
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
};

void setup_world(World *world) {
	world->materials = {
		Material {
			.color    = { 0.3f, 0.4f, 0.4f },
			.emissive = { 0.3f, 0.4f, 0.4f }
		},
		Material {
			.color = { 0.5f, 0.2f, 0.2f },
			.emissive = {}
		},
		Material {
			.color = { 0.1f, 0.2f, 0.8f },
			.emissive = {}
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
			.pos = { 0, 0, 1 },
			.r = 5,
			.mat_index = 2
		}
	};
}

struct Ray {
	vec3 origin;
	vec3 dir;
};

struct Color {
	u8 r;
	u8 g;
	u8 b;
};

const float tolerance = 0.001;

float intersect_plane(Plane plane, Ray ray) {
	float denom = dot(plane.normal, ray.dir);
	// ignore if plane normal is pointing in wrong direction
//	if (denom < -tolerance || denom > tolerance) {
	if (denom < -tolerance) {
		return (-dot(plane.normal, ray.origin) - plane.d) / denom;
	}
	return -1;
}

float intersect_sphere(Sphere sphere, Ray ray) {
	ray.origin = ray.origin - sphere.pos;

	float a = 1; // dot(ray.dir, ray.dir);
	float b = 2 * dot(ray.dir, ray.origin);
	float c = dot(ray.origin, ray.origin) - sphere.r * sphere.r;
	float D = b*b - 4*a*c;

	if (D <= tolerance)  return -1;

	float Droot = sqrt(D);
	float result1 = (-b + Droot)/(2 * a);
	float result2 = (-b - Droot)/(2 * a);

	return result1 > result2 ? result2 : result1;
}

vec3 ray_cast(World *world, Ray ray) {
	int mat_index = 0;
	float min_distance = MAXFLOAT;

	// first bounce 
	for (int bounce = 0; bounce < 2; ++bounce) {
		for (auto& plane : world->planes) {
			float distance = intersect_plane(plane, ray);
			if (distance > 0 && distance < min_distance) {
				mat_index = plane.mat_index;
				min_distance = distance;
			}
		}

		for (auto& sphere : world->spheres) {
			float distance = intersect_sphere(sphere, ray);
			if (distance > 0 && distance < min_distance) {
				mat_index = sphere.mat_index;
				min_distance = distance;
			}
		}
	}

	return world->materials[mat_index].color;
}

Color vec_to_color(vec3 v) {
	return Color {
		u8(255 * v.x),
		u8(255 * v.y),
		u8(255 * v.z),
	};
}

int main() {
	World world;
	setup_world(&world);

	s32 image_width  = 720;
	s32 image_height = 480;

	vec3 camera_pos = { 0, 10, 1 };
	vec3 look_at = { 0, 0, 0 };
	vec3 camera_dir = norm(camera_pos - look_at);

	vec3 up = { 0, 0, 1 };
	vec3 camera_right = cross(up, camera_dir);
	vec3 camera_up = cross(camera_dir, camera_right);

	float x_scale = 1.0f;
	float y_scale = float(image_height) / float(image_width);
	if (image_height > image_width) {
		x_scale = float(image_width) / float(image_height);
		y_scale = 1.0f;
	}

	std::vector<Color> buf;
	buf.resize(image_width * image_height);

	vec3 lens_pos = camera_pos - 0.5 * camera_dir;

	for (int y = 0; y < image_height; ++y) {
		float fy = 1.0f - float(y) / float(image_height-1) * 2;
		fy *= y_scale;
		int pos_in_buffer = y * image_width;
		for (int x = 0; x < image_width; ++x) {
			float fx = -1.0f + float(x) / float(image_width-1) * 2;
			fx *= x_scale;

			vec3 global_pos = fx * camera_right + fy * camera_up + lens_pos;
			Ray ray;
			ray.dir = norm(global_pos - camera_pos);
			ray.origin = camera_pos;
			vec3 color = ray_cast(&world, ray);

			buf[pos_in_buffer + x] = vec_to_color(color);
		}
	}

	stbi_write_png("test.png", image_width, image_height, sizeof(Color), buf.data(), image_width * sizeof(Color));

	return 0;
}

