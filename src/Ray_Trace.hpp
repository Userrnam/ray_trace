#pragma once

#include "World.hpp"


void seed(int);

vec3 ray_color(World *world, const Ray& ray, int sample_count, int bounce_count, int prev_count=0, vec3 *sum=nullptr);

