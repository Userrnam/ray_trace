#include "ray_trace.hpp"


const float tolerance = 0.001;

float intersect_plane(Ray ray, Plane plane) {
	float denom = dot(plane.normal, ray.dir);
	// ignore if plane normal is pointing in wrong direction
//	if (denom < -tolerance || denom > tolerance) {
	if (denom < -tolerance) {
		return (-dot(plane.normal, ray.origin) - plane.d) / denom;
	}
	return -1;
}

float intersect_sphere(Ray ray, Sphere sphere) {
	ray.origin = ray.origin - sphere.pos;

	float b_half = dot(ray.dir, ray.origin);
	float c = dot(ray.origin, ray.origin) - sphere.r * sphere.r;
	float D = b_half * b_half - c;

	if (D <= tolerance)  return -1;

	return -b_half - sqrt(D);
}

// https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/ray-triangle-intersection-geometric-solution
float intersect_triangle(Ray ray, vec3 v0, vec3 v1, vec3 v2, vec3& N) {
	// compute plane's normal
    vec3 v0v1 = v1 - v0; 
    vec3 v0v2 = v2 - v0; 

    // no need to normalize
    N = cross(v0v1, v0v2);
 
    // Step 1: finding P
 
    // check if ray and plane are parallel
    float n_dot_dir = dot(N, ray.dir);
    if (fabs(n_dot_dir) < tolerance) {
        return -1;
	}
 
    float d = -dot(N, v0);
    float t = -(dot(N, ray.origin) + d) / n_dot_dir; 
 
    // check if the triangle is in behind the ray
    if (t < 0)  return t;
 
    // compute the intersection point
    vec3 P = ray.origin + t * ray.dir; 
 
    vec3 C; // vector perpendicular to triangle's plane 
 
    // edge 0
    vec3 edge0 = v1 - v0;
    vec3 vp0 = P - v0;
    C = cross(edge0, vp0);
    if (dot(N, C) < 0) return -1;
 
    // edge 1
    vec3 edge1 = v2 - v1;
    vec3 vp1 = P - v1;
    C = cross(edge1, vp1);
    if (dot(N, C) < 0) return -1;
 
    // edge 2
    vec3 edge2 = v0 - v2; 
    vec3 vp2 = P - v2; 
    C = cross(edge2, vp2);
    if (dot(N, C) < 0) return -1;

	N = norm(N);

	if (n_dot_dir > 0) {
		N = -N;
	}
 
    return t;
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
		float distance = intersect_sphere(ray, sphere);
		if (distance > 0 && distance < min_distance) {
			hit = true;
			mat = sphere.mat_index;
			min_distance = distance;

			pos = distance * ray.dir + ray.origin;
			normal = norm(pos - sphere.pos);
		}
	}

	for (int i = 0; i < world->triangle_indices.size(); i += 3) {
		vec3 N;
		float distance = intersect_triangle(ray,
			world->triangle_vertices[world->triangle_indices[i+0]],
			world->triangle_vertices[world->triangle_indices[i+1]],
			world->triangle_vertices[world->triangle_indices[i+2]],
			N
		);
		if (distance > 0 && distance < min_distance) {
			hit = true;
			mat = world->triangle_materials[i / 3];;
			min_distance = distance;

			pos = distance * ray.dir + ray.origin;
			normal = N;
		}
	}

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
		assert(cos_att > 0.0f && cos_att <= 1.0f);
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

vec3 ray_color(World *world, Ray ray, int sample_count, int bounce_count) {
	vec3 res = {};

	for (int sample = 0; sample < sample_count; ++sample) {
		res += ray_bounce(world, ray, bounce_count);
	}

	return 1.0 / float(sample_count) * res;
}

void seed(int k) {
    state = k;
}
