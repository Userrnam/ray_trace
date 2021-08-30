#pragma once

#include <inttypes.h>
#include <math.h>

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t  s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;


struct vec3 {
	float x, y, z;
};

inline vec3 operator+(vec3 a, vec3 b) {
	return vec3 {
		a.x + b.x,
		a.y + b.y,
		a.z + b.z
	};
}

inline vec3 operator-(vec3 a, vec3 b) {
	return vec3 {
		a.x - b.x,
		a.y - b.y,
		a.z - b.z
	};
}

inline vec3 operator-(vec3 a) {
	return vec3 {
		-a.x,
		-a.y,
		-a.z
	};
}

inline vec3 operator*(float c, vec3 a) {
	return vec3 {
		c * a.x,
		c * a.y,
		c * a.z,
	};
}

inline float dot(vec3 a, vec3 b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline vec3 cross(vec3 a, vec3 b) {
	return {
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x
	};
}

float length(vec3 a) {
	return sqrt(dot(a, a));
}

inline vec3 norm(vec3 a) {
	auto l = length(a);
	return {
		a.x / l,
		a.y / l,
		a.z / l,
	};
}

