
uint state;
inline uint xor_shift_32() {
	uint x = state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	state = x;
	return x;
}

float rand() {
	return (float)(xor_shift_32() % 1000) / 1000.0f;
}

float3 rand_vec() {
	float3 res;
	res.x = -1.0f + (float)(xor_shift_32() % 1000) / 500.0f;
	res.y = -1.0f + (float)(xor_shift_32() % 1000) / 500.0f;
	res.z = -1.0f + (float)(xor_shift_32() % 1000) / 500.0f;
	return res;
}


typedef struct {
    float3 origin;
    float3 dir;
    float3 invdir;
} Ray;

float4 ray_bounce(World *world, Ray ray, int bounce_count, bool first_bounce) {
	if (bounce_count == 0) {
		return (float4)(0);
	}

	float3 pos, normal;
	int mat_index;
	bool hit_from_inside;

	if (ray_cast(world, ray, pos, normal, mat_index, hit_from_inside)) {
        Material mat = world->materials.data[mat_index];

		ray.origin = pos;

		float cos_theta = -dot(normal, ray.dir); // normal and ray dir have length 1
		float cos_att = 1; 
		if (hit_from_inside) {
			if (mat.refractiveness == 0) {
				return (float4)(0);
			}
            float sin_theta = sqrt(1.0 - cos_theta*cos_theta);
			float refraction_ratio = mat.n;
			if (refraction_ratio * sin_theta > 1.0 || probability_value(reflectance(cos_theta, refraction_ratio))) {
				ray.dir = reflect(ray.dir, normal);
			} else {
				ray.dir = refract(ray.dir, normal, refraction_ratio);
			}
		} else if (mat.refractiveness > 0) {
            float sin_theta = sqrt(1.0 - cos_theta*cos_theta);
			float refraction_ratio = 1.0f / mat.n;
			if (refraction_ratio * sin_theta > 1.0 || probability_value(reflectance(cos_theta, refraction_ratio))) {
				ray.dir = reflect(ray.dir, normal);
			} else {
				ray.dir = refract(ray.dir, normal, refraction_ratio);
			}
		} else {
			cos_att = clamp(cos_theta, 0.0f, 1.0f);
			if (first_bounce)  cos_att = 1;

			// update ray
			float3 reflection_ray = reflect(ray.dir, normal);
			ray.dir = lerp(norm(rand_vec()), reflection_ray, mat.specular);
			if (dot(ray.dir, normal) < 0) {
				ray.dir = -ray.dir;
			}
		}

		ray.invdir = 1.0f / ray.dir;

		return mat.emissive + cos_att * mat.color * ray_bounce(world, ray, bounce_count - 1, false);
	}

	return world->materials.data[0].emissive;
}

float4 ray_color(World *world, Ray ray, int sample_count, int bounce_count, int prev_count, float4 *sum) {
	for (int sample = 0; sample < sample_count; ++sample) {
		*sum += ray_bounce(world, ray, bounce_count, true);
	}

	return *sum / (float)(sample_count + prev_count);
}

Ray get_ray(Camera *camera, int x, int y) {
    float v = ((float) y / (float) camera->height) * 2.0f - 1.0f;  // [-1; 1]
    float u = ((float) x / (float) camera->width) * 2.0f - 1.0f;   // [-1; 1]
    u += 0.5/camera->width  * rand();
    v += 0.5/camera->height * rand();

    Ray ray;
    ray.origin = camera->pos;
    ray.dir = norm(camera->center + u * camera->horizontal + v * camera->vertical - camera->pos);
    ray.invdir = 1.0f / ray.dir;
    return ray;
}
