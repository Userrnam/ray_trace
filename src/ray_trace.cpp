#include "ray_trace.hpp"


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

void seed(int k) {
    state = k;
}

const float tolerance = 0.001;

float intersect_plane(const Ray& ray, const Plane& plane) {
	float denom = dot(plane.normal, ray.dir);
	// ignore if plane normal is pointing in wrong direction
//	if (denom < -tolerance || denom > tolerance) {
	if (denom < -tolerance) {
		return (-dot(plane.normal, ray.origin) - plane.d) / denom;
	}
	return -1;
}

float intersect_sphere(const Ray& ray, const Sphere& sphere, bool& inside) {
	vec3 a = ray.origin - sphere.pos;

	// if ray is inside sphere b_half is < 0
	float b_half = dot(ray.dir, a);
	float c = dot(a, a) - sphere.r * sphere.r;
	float D = b_half * b_half - c;

	if (D <= tolerance)  return -1;

	float rD = sqrt(D);
	float t = -b_half - sqrt(D);

	// ray is inside sphere. This can happen in refraction
	if (t < 0) {
		inside = true;
		return -b_half + sqrt(D);
	}
	inside = false;
	return t;
}

// https://ru.wikipedia.org/wiki/Алгоритм_Моллера_—_Трумбора
float intersect_triangle(const Ray& ray, const vec3& v0, const vec3& v1, const vec3& v2) {
    vec3 e1 = v1 - v0;
    vec3 e2 = v2 - v0;

	// calculate triple product of e1, e2 and ray.dir
    vec3 pvec = cross(ray.dir, e2);
    float det = dot(e1, pvec);

	// ray and plane are parallel
    if (det < tolerance && det > -tolerance) {
        return -1;
    }

    float inv_det = 1 / det;
    vec3 tvec = ray.origin - v0;
    float u = dot(tvec, pvec) * inv_det;
    if (u < 0 || u > 1) {
        return -1;
    }

    vec3 qvec = cross(tvec, e1);
    float v = dot(ray.dir, qvec) * inv_det;
    if (v < 0 || u + v > 1) {
        return -1;
    }
    return dot(e2, qvec) * inv_det;
}

bool ray_cast(World *world, Ray ray, vec3& pos, vec3& normal, int& mat) {
	float min_distance = MAXFLOAT;
	bool hit = false;

	mat = 0;
	for (auto& plane : world->planes) {
		float distance = intersect_plane(ray, plane);
		if (distance > 0 && distance < min_distance) {
			hit = true;
			mat = plane.mat_index;
			min_distance = distance;

			pos = distance * ray.dir + ray.origin;
			normal = plane.normal;
		}
	}

	for (auto& sphere : world->spheres) {
		bool inside;
		float distance = intersect_sphere(ray, sphere, inside);
		if (!inside && distance > 0 && distance < min_distance) {
			hit = true;
			mat = sphere.mat_index;
			min_distance = distance;

			pos = distance * ray.dir + ray.origin;
			normal = pos - sphere.pos;
		}
	}

	for (int i = 0; i < world->triangle_indices.size(); i += 3) {
		float distance = intersect_triangle(ray,
			world->triangle_vertices[world->triangle_indices[i+0]],
			world->triangle_vertices[world->triangle_indices[i+1]],
			world->triangle_vertices[world->triangle_indices[i+2]]
		);

		vec3 N = cross(
			world->triangle_vertices[world->triangle_indices[i+0]] - world->triangle_vertices[world->triangle_indices[i+1]],
			world->triangle_vertices[world->triangle_indices[i+0]] - world->triangle_vertices[world->triangle_indices[i+2]]
		);

		// triangle normal is looking in wrong direction.
		if (dot(N, ray.dir) > 0) {
			N = -N;
			// without this noise will appear
			distance -= tolerance;
		}
		if (distance > 0 && distance < min_distance) {
			hit = true;
			mat = world->triangle_materials[i / 3];
			min_distance = distance;

			pos = distance * ray.dir + ray.origin;
			normal = N;
		}
	}

	normal = norm(normal);

	return hit;
}

vec3 ray_bounce(World *world, Ray ray, int bounce_count, bool first_bounce = true) {
	if (bounce_count == 0) {
		return vec3(0, 0, 0);
	}

	vec3 pos, normal;
	int mat_index;

	if (ray_cast(world, ray, pos, normal, mat_index)) {
		float cos_att = -dot(normal, ray.dir); // normal and ray dir have length 1
		// cos_att may be less then 0 or grater than 1 due to floating point error
		cos_att = clamp(cos_att, 0, 1);
		if (first_bounce)  cos_att = 1;

		const auto& mat = world->materials[mat_index];

		// update ray
		vec3 reflection_ray = reflect(ray.dir, normal);
		ray.origin = pos;
		ray.dir = lerp(norm(rand_vec()), reflection_ray, mat.specular);
		if (dot(ray.dir, normal) < 0) {
			ray.dir = -ray.dir;
		}
		return mat.emissive + cos_att * mul(mat.color, ray_bounce(world, ray, bounce_count - 1, false));
	}

	return world->materials[0].emissive;
}

vec3 ray_color(World *world, const Ray& ray, int sample_count, int bounce_count, int prev_count, vec3 *sum) {
	vec3 res = {};
	if (sum == nullptr) {
		sum = &res;
	}
	for (int sample = 0; sample < sample_count; ++sample) {
		*sum += ray_bounce(world, ray, bounce_count);
	}

	return 1.0 / float(sample_count+prev_count) * *sum;
}
