#ifndef RAY_TRACE_HCL
#define RAY_TRACE_HCL

#include "data_structures.hcl"

float4 ray_color(World *world, Ray ray, int sample_count, int bounce_count, int prev_count, float4 *sum);
bool intersect_bounding_box(Bounding_Box *bounding_box, Ray ray);

#endif
