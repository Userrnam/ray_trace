#pragma once

#include "Math.hpp"

extern u32 state;
inline u32 xor_shift_32() {
	u32 x = state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	state = x;
	return x;
}

inline vec3 rand_vec() {
	vec3 res = { 1, 1, 1 };
	while (dot(res, res) > 1) {
		res.x = -1.0f + float(xor_shift_32() % 1000) / 500.0f;
		res.y = -1.0f + float(xor_shift_32() % 1000) / 500.0f;
		res.z = -1.0f + float(xor_shift_32() % 1000) / 500.0f;
	}
	return norm(res);
}

// returns 1 with probability p
inline bool probability_value(float p) {
	return p > float(xor_shift_32() % 10000) / 10000;
}
