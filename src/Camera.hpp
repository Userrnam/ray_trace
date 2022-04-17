#pragma once

#include "ray_trace.hpp"

class Camera {
	vec3 _pos;
	vec3 _dir;
	vec3 _center;
	vec3 _vertical;
	vec3 _horizontal;
	int _width, _height;
	float _lense_distance;
public:
	int get_width()  { return _width; }
	int get_height() { return _height; }

	void move(vec3 d) {
		vec3 _up = { 0, 0, 1 };
		vec3 right = cross(_up, _dir);
		_pos += d.x * right;
		_pos += d.y * _dir;

		_center = _pos - _lense_distance * _dir;
	}
	void rotate_z(float delta) {
		vec3 _up = { 0, 0, 1 };
		vec3 right = cross(_up, _dir);
		vec3 up = cross(_dir, right);

		// this is incorrect and dumb
		_dir += delta * right;
		_dir = norm(_dir);

		_center = _pos - _lense_distance * _dir;
		_vertical   = 0.5 * up;
		_horizontal = 0.5 * (float(_width) / _height) * right;
	}

	void create(int width, int height, vec3 pos, vec3 look_at, float lense_distance) {
		_lense_distance = lense_distance;
		_pos = pos;
		_dir = norm(_pos - look_at);
		vec3 _up = { 0, 0, 1 };
		vec3 right = cross(_up, _dir);
		vec3 up = cross(_dir, right);

		_center = _pos - lense_distance * _dir;
		_vertical   = 0.5 * up;
		_horizontal = 0.5 * (float(width) / height) * right;

		_width = width;
		_height = height;
	}

	Ray get_ray(float u, float v) {
		u += 1.0/_width * float(rand()) / RAND_MAX;
		v += 1.0/_height * float(rand()) / RAND_MAX;

		Ray rey;
		rey.origin = _pos;
		rey.dir = norm(_center + u * _horizontal + v * _vertical - _pos);
		return rey;
	}
};
