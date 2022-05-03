#include "random.hcl"

uint state = 45233906;

void seed(uint k) {
    state = k;
}

uint xor_shift_32() {
	uint x = state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	state = x;
	return x;
}

uint xor_shift_32_2(uint x) {
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	return x;
}

float rand() {
    return (float)(xor_shift_32() % 1000) / 1000.0f;
}

float rand2(uint* state) {
    *state = xor_shift_32_2(*state);
    return (float)(*state % 1000) / 1000.0f;
}

float3 rand_vec() {
	float3 res;
    res.x = -1.0f + 2.0f * rand();
    res.y = -1.0f + 2.0f * rand();
    res.z = -1.0f + 2.0f * rand();
	return res;
}

bool probability_value(float p) {
	return p > rand();
}
