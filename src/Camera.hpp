#pragma once

#include "ray_trace.hpp"

struct Camera {
	vec3 pos;
	vec3 dir;
	vec3 center;
	vec3 vertical;
	vec3 horizontal;

	void create(float width, float height, vec3 _pos, vec3 look_at, float lense_distance) {
		pos = _pos;
		dir = norm(pos - look_at);
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
