#include "Ray_Trace.hpp"

#include <float.h>

#include "Random.hpp"

u32 state;

float reflectance(float cosine, float ref_idx) {
	// Use Schlick's approximation for reflectance.
	float r0 = (1-ref_idx) / (1+ref_idx);
	r0 = r0*r0;
	return r0 + (1-r0)*pow((1 - cosine),5);
}

void seed(int k) {
    state = k;
}

const float tolerance = 0.0001f;

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

	inside = c < 0;

	float rD = sqrt(D);
	float t = -b_half - sqrt(D);
	// this can happen if ray is inside the sphere.
	if (t < 0) {
		return -b_half + sqrt(D);
	}
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

bool ray_cast(World *world, Ray ray, vec3& pos, vec3& normal, int& mat, bool& hit_from_inside) {
	float min_distance = FLT_MAX;
	bool hit = false;

	mat = 0;
	hit_from_inside = false;
	for (auto& plane : world->planes) {
		float distance = intersect_plane(ray, plane);
		if (distance > tolerance && distance < min_distance) {
			hit = true;
			mat = plane.mat_index;
			min_distance = distance;

			pos = distance * ray.dir + ray.origin;
			normal = plane.normal;
			hit_from_inside = false;
		}
	}

	for (auto& sphere : world->spheres) {
		bool inside;
		float distance = intersect_sphere(ray, sphere, inside);
		if (distance > tolerance && distance < min_distance) {
			hit_from_inside = inside;

			hit = true;
			mat = sphere.mat_index;
			min_distance = distance;

			pos = distance * ray.dir + ray.origin;
			normal = pos - sphere.pos;
		}
	}

	const Obj_File& obj = world->obj;
	for (int mesh_index = 0; mesh_index < world->mesh_indices.size(); ++mesh_index) {
		const Mesh& mesh = obj.meshes[world->mesh_indices[mesh_index]];

		float distance;
		int triangle_index;
		if (!mesh.bvh.intersect(&obj, mesh.vi_first, ray, &triangle_index, &distance)) {
			continue;
		}

		vec3 N = obj.normals[obj.normal_indices[mesh.ni_first + 3 * triangle_index]];

		bool inside = false;

		// triangle normal is looking in wrong direction.
		if (dot(N, ray.dir) > 0) {
			inside = true;
			// N = -N;
			// without this noise will appear
			distance -= tolerance;
		}

		if (distance > tolerance && distance < min_distance) {
			hit = true;
			mat = mesh.material_index;
			min_distance = distance;

			pos = distance * ray.dir + ray.origin;
			normal = N;
			hit_from_inside = inside;
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
	bool hit_from_inside;

	if (ray_cast(world, ray, pos, normal, mat_index, hit_from_inside)) {
		assert(mat_index);
		const auto& mat = world->materials[mat_index];

		ray.origin = pos;

		float cos_theta = -dot(normal, ray.dir); // normal and ray dir have length 1
		float cos_att = 1; 
		if (hit_from_inside) {
			if (mat.refractiveness == 0) {
				// this can happen near corners due to floating point error.
				return {};
			}
            float sin_theta = sqrt(1.0 - cos_theta*cos_theta);
			float refraction_ratio = mat.n;
			if (refraction_ratio * sin_theta > 1.0 || probability_value(reflectance(cos_theta, refraction_ratio))) {
				ray.dir = reflect(ray.dir, normal);
			} else {
				ray.dir = refract(ray.dir, normal, refraction_ratio);
			}
		} else if (mat.refractiveness > 0) {
            float sin_theta = sqrt(1.0 - cos_theta*cos_theta);
			float refraction_ratio = 1.0f / mat.n;
			if (refraction_ratio * sin_theta > 1.0 || probability_value(reflectance(cos_theta, refraction_ratio))) {
				ray.dir = reflect(ray.dir, normal);
			} else {
				ray.dir = refract(ray.dir, normal, refraction_ratio);
			}
			cos_att = 1;
			bounce_count++;
		} else {
			cos_att = clamp(cos_theta, 0, 1);
			if (first_bounce)  cos_att = 1;

			// update ray
			vec3 reflection_ray = reflect(ray.dir, normal);
			ray.dir = lerp(rand_vec(), reflection_ray, mat.specular);
			if (dot(ray.dir, normal) < 0) {
				ray.dir = -ray.dir;
			}
		}

		ray.update();

		if (mat.emissive) {
			return mat.color;
		}

		return cos_att * mul(mat.color, ray_bounce(world, ray, bounce_count - 1, false));
	}

	return world->materials[0].color;
}

vec3 ray_color(World *world, const Ray& ray, int sample_count, int bounce_count, int prev_count, vec3 *sum) {
	vec3 res = {};
	if (sum == nullptr) {
		sum = &res;
	}
	for (int sample = 0; sample < sample_count; ++sample) {
		*sum += ray_bounce(world, ray, bounce_count);
	}

	vec3 color = 1.0f / float(sample_count + prev_count) * *sum;

	float gamma_correction = 1.0f / 2.2f;
	color.x = pow(color.x, gamma_correction);
	color.y = pow(color.y, gamma_correction);
	color.z = pow(color.z, gamma_correction);

	return color;
}

