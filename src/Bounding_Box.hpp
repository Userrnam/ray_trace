#pragma once

#include "Math.hpp"

struct Bounding_Box {
	vec3 points[2];

	bool intersect(const Ray& ray) const;
};

