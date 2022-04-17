#pragma once

#include "world.hpp"

struct Ray {
	vec3 origin;
	vec3 dir;
};

void seed(int);

vec3 ray_color(World *world, Ray ray, int sample_count, int bounce_count, int prev_count=0, vec3 *sum=nullptr);
