uint state;
inline uint xor_shift_32() {
	uint x = state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	state = x;
	return x;
}

float3 rand_vec() {
	float3 res;
	res.x = -1.0f + (float)(xor_shift_32() % 1000) / 500.0f;
	res.y = -1.0f + (float)(xor_shift_32() % 1000) / 500.0f;
	res.z = -1.0f + (float)(xor_shift_32() % 1000) / 500.0f;
	return res;
}

// returns 1 with probability p
bool probability_value(float p) {
	return p > (float)(xor_shift_32() % 10000) / 10000;
}

float reflectance(float cosine, float ref_idx) {
	// Use Schlick's approximation for reflectance.
	float r0 = (1-ref_idx) / (1+ref_idx);
	r0 = r0*r0;
	return r0 + (1-r0)*pow((1 - cosine),5);
}

void seed(int k) {
    state = k;
}

const float tolerance = 0.001;

typedef struct {
	float3 origin;
	float3 dir;
	float3 invdir;
} Ray;

// https://ru.wikipedia.org/wiki/Алгоритм_Моллера_—_Трумбора
float intersect_triangle(Ray ray, float3 v0, float3 v1, float3 v2) {
    float3 e1 = v1 - v0;
    float3 e2 = v2 - v0;

	// calculate triple product of e1, e2 and ray.dir
    float3 pvec = cross(ray.dir, e2);
    float det = dot(e1, pvec);

	// ray and plane are parallel
    if (det < tolerance && det > -tolerance) {
        return -1;
    }

    float inv_det = 1 / det;
    float3 tvec = ray.origin - v0;
    float u = dot(tvec, pvec) * inv_det;
    if (u < 0 || u > 1) {
        return -1;
    }

    float3 qvec = cross(tvec, e1);
    float v = dot(ray.dir, qvec) * inv_det;
    if (v < 0 || u + v > 1) {
        return -1;
    }
    return dot(e2, qvec) * inv_det;
}

typedef struct {
	float3 *data;
	int count;
} f3array;

typedef struct {
	int *data;
	int count;
} iarray;

typedef struct {
	float3 color;
	float3 emissive;
	float specular; // 1 - very reflective, 0 - not reflective
	float refractiveness;
	float n; // refraction coefficient
} Material;

typedef struct {
	iarray vertex_indices;
	iarray normal_indices;
	int material_index;
} Mesh;

typedef struct {
	Material *data;
	int count;
} MaterialArray;

typedef struct {
	Mesh *data;
	int count;
} MeshArray;

typedef struct {
	MeshArray meshes;
	MaterialArray materials;
	f3array vertices;
	f3array normals;
} World;

bool ray_cast(World *world, Ray ray, float3* pos, float3* normal, int* mat, bool* hit_from_inside) {
	float min_distance = FLT_MAX;
	bool hit = false;

	*mat = 0;
	*hit_from_inside = false;

	for (int mesh_index = 0; mesh_index < world->meshes.count; ++mesh_index) {
		Mesh mesh = world->meshes.data[mesh_index];
		iarray triangle_indices;
		// TODO
		// std::vector<int> triangle_indices;
		// if (!mesh.bvh.intersect(ray, triangle_indices)) {
		// 	continue;
		// }

		for (int i = 0; i < triangle_indices.count; ++i) {
			int triangle_index = triangle_indices.data[i];
			
			float distance = intersect_triangle(ray,
				world->vertices.data[mesh.vertex_indices.data[3 * triangle_index + 0]],
				world->vertices.data[mesh.vertex_indices.data[3 * triangle_index + 1]],
				world->vertices.data[mesh.vertex_indices.data[3 * triangle_index + 2]]
			);

			float3 N = world->normals.data[mesh.normal_indices.data[3 * triangle_index]];

			bool inside = false;

			// triangle normal is looking in wrong direction.
			if (dot(N, ray.dir) > 0) {
				inside = true;
				// N = -N;
				// without this noise will appear
				distance -= tolerance;
			}

			if (distance > 0 && distance < min_distance) {
				hit = true;
				*mat = mesh.material_index;
				min_distance = distance;

				*pos = distance * ray.dir + ray.origin;
				*normal = N;
				*hit_from_inside = inside;
			}
		}
	}

	*normal = normalize(*normal);

	return hit;
}

float3 ray_bounce(World *world, Ray ray, int bounce_count, bool first_bounce) {
	if (bounce_count == 0) {
		return (float3)(0);
	}

	float3 pos, normal;
	int mat_index;
	bool hit_from_inside;

	if (ray_cast(world, ray, &pos, &normal, &mat_index, &hit_from_inside)) {
		Material *mat = &world->materials.data[mat_index];

		ray.origin = pos;

		float cos_theta = -dot(normal, ray.dir); // normal and ray dir have length 1
		float cos_att = 1; 
		if (hit_from_inside) {
			if (mat->refractiveness == 0) {
				return (float3)(0);
			}
            float sin_theta = sqrt(1.0 - cos_theta*cos_theta);
			float refraction_ratio = mat->n;
			if (refraction_ratio * sin_theta > 1.0 || probability_value(reflectance(cos_theta, refraction_ratio))) {
				ray.dir = reflect(ray.dir, normal);
			} else {
				ray.dir = refract(ray.dir, normal, refraction_ratio);
			}
		} else if (mat->refractiveness > 0) {
            float sin_theta = sqrt(1.0 - cos_theta*cos_theta);
			float refraction_ratio = 1.0f / mat->n;
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
			ray.dir = lerp(norm(rand_vec()), reflection_ray, mat->specular);
			if (dot(ray.dir, normal) < 0) {
				ray.dir = -ray.dir;
			}
		}

		ray.invdir = (float3)(1.0f) / ray.dir;

		return mat->emissive + cos_att * mat->color * ray_bounce(world, ray, bounce_count - 1, false);
	}

	return world->materials.data[0].emissive;
}

float3 ray_color(World *world, Ray ray, int sample_count, int bounce_count, int prev_count, float3 *sum) {
	float3 res = {};
	for (int sample = 0; sample < sample_count; ++sample) {
		*sum += ray_bounce(world, ray, bounce_count, true);
	}

	return 1.0f / (float)(sample_count+prev_count) * *sum;
}
