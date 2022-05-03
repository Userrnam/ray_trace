#ifndef RANDOM_HCL
#define RANDOM_HCL

void seed(uint k);

uint xor_shift_32();
uint xor_shift_32_2(uint x);

// return random value from 0 to 1
float rand();

float rand2(uint* state);

// random normalized vector
float3 rand_vec();

// returns 1 with probability p
bool probability_value(float p);

#endif
