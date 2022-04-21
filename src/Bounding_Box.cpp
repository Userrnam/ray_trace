#include "Bounding_Box.hpp"

#include <float.h>

float intersect_p(const Ray& ray, vec3 normal, float d) {
	float denom = dot(normal, ray.dir);
	// ignore if plane normal is pointing in wrong direction
//	if (denom < -tolerance || denom > tolerance) {
	if (denom < -0.001) {
		return (-dot(normal, ray.origin) - d) / denom;
	}
	return -1;
}

bool Bounding_Box::intersect(const Ray& ray) const {
	// if ray is inside bounding box return true
	if (points[0].x < ray.origin.x && ray.origin.x < points[1].x &&
		points[0].y < ray.origin.y && ray.origin.y < points[1].y &&
		points[0].z < ray.origin.z && ray.origin.z < points[1].z) {
		return true;
	}

	// right
	if (ray.dir.x < -0.001) {
		float t = -(ray.origin.x - points[1].x) / ray.dir.x;

		vec3 p = ray.origin + t * ray.dir;
		if (t > 0 && points[0].y <= p.y && p.y <= points[1].y &&
			points[0].z <= p.z && p.z <= points[1].z) {
			return true;
		}
	}
	// back
	if (ray.dir.y < -0.001) {
		float t = -(ray.origin.y - points[1].y) / ray.dir.y;

		vec3 p = ray.origin + t * ray.dir;
		if (t > 0 && points[0].x <= p.x && p.x <= points[1].x &&
			points[0].z <= p.z && p.z <= points[1].z) {
			return true;
		}
	}
	// top
	if (ray.dir.z < -0.001) {
		float t = -(ray.origin.z - points[1].z) / ray.dir.z;

		vec3 p = ray.origin + t * ray.dir;
		if (t > 0 && points[0].x <= p.x && p.x <= points[1].x &&
			points[0].y <= p.y && p.y <= points[1].y) {
			return true;
		}
	}

	// left
	if (ray.dir.x > 0.001) {
		float t = -(ray.origin.x - points[0].x) / ray.dir.x;

		vec3 p = ray.origin + t * ray.dir;
		if (t > 0 && points[0].y <= p.y && p.y <= points[1].y &&
			points[0].z <= p.z && p.z <= points[1].z) {
			return true;
		}
	}
	// front
	if (ray.dir.y > 0.001) {
		float t = -(ray.origin.y - points[0].y) / ray.dir.y;

		vec3 p = ray.origin + t * ray.dir;
		if (t > 0 && points[0].x <= p.x && p.x <= points[1].x &&
			points[0].z <= p.z && p.z <= points[1].z) {
			return true;
		}
	}
	// bottom
	if (ray.dir.z > 0.001) {
		float t = -(ray.origin.z - points[0].z) / ray.dir.z;

		vec3 p = ray.origin + t * ray.dir;
		if (t > 0 && points[0].x <= p.x && p.x <= points[1].x &&
			points[0].y <= p.y && p.y <= points[1].y) {
			return true;
		}
	}
	return false;
}

