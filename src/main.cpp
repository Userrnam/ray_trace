#include "math.hpp"

#include <stb_image.h>
#include <vector>
#include <stdlib.h>
#include <limits.h>

struct Material {
	vec3 color;
	vec3 emissive;
	float specular; // 1 - very reflective, 0 - not reflective
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
			.color    = { 0.3f, 0.4f, 0.8f },
			.emissive = { 1.1f, 1.1f, 1.2f },
			// .emissive = {},
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
			.mat_index = 0
		},
		Sphere {
			.pos = { 5, 2, 2.1 },
			.r = 2,
			.mat_index = 3
		},
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

	float b_half = dot(ray.dir, ray.origin);
	float c = dot(ray.origin, ray.origin) - sphere.r * sphere.r;
	float D = b_half * b_half - c;

	if (D <= tolerance)  return -1;

	return -b_half - sqrt(D);

/*
	float a = 1; // dot(ray.dir, ray.dir);
	float b_half = dot(ray.dir, ray.origin);
	float c = dot(ray.origin, ray.origin) - sphere.r * sphere.r;
	float D = b_half * b_half - a*c;

	if (D <= tolerance)  return -1;

	return (-b_half - sqrt(D)) / a;
*/
}

u32 state;
inline u32 xor_shift_32() {
	u32 x = state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	state = x;
	return x;
}

vec3 rand_vec() {
	vec3 res;
	res.x = -1.0f + float(xor_shift_32() % 1000) / 500.0f;
	res.y = -1.0f + float(xor_shift_32() % 1000) / 500.0f;
	res.z = -1.0f + float(xor_shift_32() % 1000) / 500.0f;
	return res;
}

bool ray_cast(World *world, Ray ray, vec3& pos, vec3& normal, int& mat) {
	float min_distance = MAXFLOAT;
	bool hit = false;

	mat = 0;
	for (auto& plane : world->planes) {
		float distance = intersect_plane(plane, ray);
		if (distance > 0 && distance < min_distance) {
			hit = true;
			mat = plane.mat_index;
			min_distance = distance;

			pos = distance * ray.dir + ray.origin;
			normal = plane.normal;
		}
	}

	for (auto& sphere : world->spheres) {
		float distance = intersect_sphere(sphere, ray);
		if (distance > 0 && distance < min_distance) {
			hit = true;
			mat = sphere.mat_index;
			min_distance = distance;

			pos = distance * ray.dir + ray.origin;
			normal = norm(pos - sphere.pos);
		}
	}

	return hit;
}

vec3 ray_color(World *world, Ray _ray) {
	const int bounce_count = 8;
	const int sample_count = 200;

	vec3 res = {};

	for (int sample = 0; sample < sample_count; ++sample) {
		Ray ray = _ray;
		vec3 color = {};
		vec3 attenuation = { 1, 1, 1 };

		for (int bounce = 0; bounce < bounce_count; ++bounce) {
			vec3 pos, normal;
			int mat;

			if (ray_cast(world, ray, pos, normal, mat)) {
				float cos_att = -dot(normal, ray.dir); // normal and ray dir have length 1
				assert(cos_att > 0.0f && cos_att <= 1.0f);
				if (!bounce) cos_att = 1;

				color += mul(attenuation, cos_att * world->materials[mat].emissive);
				attenuation = mul(attenuation, cos_att * world->materials[mat].color);

				// update ray
				vec3 reflection_ray = reflect(ray.dir, normal);
				ray.origin = pos;
				ray.dir = lerp(norm(rand_vec()), reflection_ray, world->materials[mat].specular);
				if (dot(ray.dir, normal) < 0) {
					ray.dir = -ray.dir;
				}
			} else {
				color = color + mul(attenuation, world->materials[mat].emissive);
				break;
			}
		}

		res = res + color;
	}

	return 1.0 / float(sample_count) * res;
}

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

	srand(time(NULL));
	state = rand();

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

			vec3 color = ray_color(&world, ray);

			img[y * image_width + x] = vec_to_color(color);
        }
    }

	stbi_write_png("test.png", image_width, image_height, sizeof(Color), img.data(), image_width * sizeof(Color));

	return 0;
}

