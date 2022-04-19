#pragma once

#include "Ray_Trace.hpp"

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

	vec3 get_position() { return _pos; }
	vec3 get_direction() { return _dir; }

	void move(vec3 d) {
		vec3 _up = { 0, 0, 1 };
		vec3 right = cross(_up, _dir);
		_pos += d.x * right;
		_pos += d.y * _dir;
		_pos.z += d.z;

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

	void resize(int width, int height) {
		vec3 _up = { 0, 0, 1 };
		vec3 right = cross(_up, _dir);
		_horizontal = 0.5 * (float(width) / height) * right;

		_width = width;
		_height = height;
	}

	void create2(int width, int height, vec3 pos, vec3 dir, float lense_distance) {
		_dir = dir;
		_lense_distance = lense_distance;
		_pos = pos;
		vec3 _up = { 0, 0, 1 };
		vec3 right = cross(_up, _dir);
		vec3 up = cross(_dir, right);

		_center = _pos - lense_distance * _dir;
		_vertical   = 0.5 * up;
		_horizontal = 0.5 * (float(width) / height) * right;

		_width = width;
		_height = height;
	}

	void create(int width, int height, vec3 pos, vec3 look_at, float lense_distance) {
		_dir = norm(pos - look_at);

		create2(width, height, pos, _dir, lense_distance);
	}

	Ray get_ray(int x, int y) {
		float v = ((float) y / (float) _height) * 2.0f - 1.0f;  // [-1; 1]
		float u = ((float) x / (float) _width) * 2.0f - 1.0f;   // [-1; 1]
		u += 0.5/_width * float(rand()) / RAND_MAX;
		v += 0.5/_height * float(rand()) / RAND_MAX;

		Ray rey;
		rey.origin = _pos;
		rey.dir = norm(_center + u * _horizontal + v * _vertical - _pos);
		return rey;
	}
};
