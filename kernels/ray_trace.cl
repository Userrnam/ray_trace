#include "ray_trace.hcl"
#include "random.hcl"


const float tolerance = 0.001;


float intersect_sphere(Ray ray, Sphere sphere, bool* inside) {
	float3 a = ray.origin - VEC3(sphere.pos);

	// if ray is inside sphere b_half is < 0
	float b_half = dot(ray.dir, a);
	float c = dot(a, a) - sphere.r * sphere.r;
	float D = b_half * b_half - c;

	if (D <= tolerance)  return -1;

	float rD = sqrt(D);
	float t = -b_half - sqrt(D);

	// ray is inside sphere. This can happen in refraction
	if (t < 0) {
		*inside = true;
		return -b_half + sqrt(D);
	}
	*inside = false;
	return t;
}

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

bool intersect_bounding_box(Bounding_Box *bounding_box, Ray ray) {
    // if ray is inside bounding box return true
    if (bounding_box->points[0].x < ray.origin.x && ray.origin.x < bounding_box->points[1].x &&
        bounding_box->points[0].y < ray.origin.y && ray.origin.y < bounding_box->points[1].y &&
        bounding_box->points[0].z < ray.origin.z && ray.origin.z < bounding_box->points[1].z) {
        return true;
    }

    // right
    if (ray.dir.x < -0.001) {
        float t = (bounding_box->points[1].x - ray.origin.x) * ray.invdir.x;

        float3 p = ray.origin + t * ray.dir;
        if (t > 0 && bounding_box->points[0].y <= p.y && p.y <= bounding_box->points[1].y &&
            bounding_box->points[0].z <= p.z && p.z <= bounding_box->points[1].z) {
            return true;
        }
    }
    // back
    if (ray.dir.y < -0.001) {
        float t = (bounding_box->points[1].y - ray.origin.y) * ray.invdir.y;

        float3 p = ray.origin + t * ray.dir;
        if (t > 0 && bounding_box->points[0].x <= p.x && p.x <= bounding_box->points[1].x &&
            bounding_box->points[0].z <= p.z && p.z <= bounding_box->points[1].z) {
            return true;
        }
    }
    // top
    if (ray.dir.z < -0.001) {
        float t = (bounding_box->points[1].z - ray.origin.z) * ray.invdir.z;

        float3 p = ray.origin + t * ray.dir;
        if (t > 0 && bounding_box->points[0].x <= p.x && p.x <= bounding_box->points[1].x &&
            bounding_box->points[0].y <= p.y && p.y <= bounding_box->points[1].y) {
            return true;
        }
    }

    // left
    if (ray.dir.x > 0.001) {
        float t = (bounding_box->points[0].x - ray.origin.x) * ray.invdir.x;

        float3 p = ray.origin + t * ray.dir;
        if (t > 0 && bounding_box->points[0].y <= p.y && p.y <= bounding_box->points[1].y &&
            bounding_box->points[0].z <= p.z && p.z <= bounding_box->points[1].z) {
            return true;
        }
    }
    // front
    if (ray.dir.y > 0.001) {
        float t = (bounding_box->points[0].y - ray.origin.y) * ray.invdir.y;

        float3 p = ray.origin + t * ray.dir;
        if (t > 0 && bounding_box->points[0].x <= p.x && p.x <= bounding_box->points[1].x &&
            bounding_box->points[0].z <= p.z && p.z <= bounding_box->points[1].z) {
            return true;
        }
    }
    // bottom
    if (ray.dir.z > 0.001) {
        float t = (bounding_box->points[0].z - ray.origin.z) * ray.invdir.z;

        float3 p = ray.origin + t * ray.dir;
        if (t > 0 && bounding_box->points[0].x <= p.x && p.x <= bounding_box->points[1].x &&
            bounding_box->points[0].y <= p.y && p.y <= bounding_box->points[1].y) {
            return true;
        }
    }
    return false;
}

bool intersect_bvh(World* world, int bvh_first, int vi_first, Ray ray, int *triangle_index, float *t) {
    int stack[20];
    int stack_pointer = 0;
    stack[stack_pointer++] = bvh_first;

	bool intersected = false;
    while (stack_pointer) {
		BVH_Node node = world->bvh_nodes.data[stack[--stack_pointer]];
        if (!intersect_bounding_box(&node.bounding_box, ray)) {
			continue;
        }
		// it's a leaf
		if (node.triangle_count) {
			for (int i = 0; i < node.triangle_count; ++i) {
				int index = world->triangle_indices.data[node.first_triangle + i];
				float distance = intersect_triangle(ray,
					world->vertices.data[world->vertex_indices.data[vi_first + 3 * index + 0]].xyz,
					world->vertices.data[world->vertex_indices.data[vi_first + 3 * index + 1]].xyz,
					world->vertices.data[world->vertex_indices.data[vi_first + 3 * index + 2]].xyz
				);
				if (distance > tolerance && distance < *t) {
					intersected = true;
					*t = distance;
					*triangle_index = index;
				}
			}
		} else {
            stack[stack_pointer++] = node.left;
            stack[stack_pointer++] = node.right;
		}
    }
    return intersected;
}

bool ray_cast(World *world, Ray ray, float3* pos, float3* normal, int* mat, bool* hit_from_inside) {
	float min_distance = FLT_MAX;
	bool hit = false;

	*mat = 0;
	*hit_from_inside = false;

    // spheres
	for (int i = 0; i < world->spheres.count; ++i) {
        Sphere sphere = world->spheres.data[i];
		bool inside;
		float distance = intersect_sphere(ray, sphere, &inside);
		if (distance > 0 && distance < min_distance) {
			*hit_from_inside = inside;

			hit = true;
			*mat = sphere.mat_index;
			min_distance = distance;

			*pos = distance * ray.dir + ray.origin;
			*normal = *pos - VEC3(sphere.pos);
		}
	}

    // meshes
	for (int mesh_index = 0; mesh_index < world->mesh_indices.count; ++mesh_index) {
		Mesh mesh = world->meshes.data[world->mesh_indices.data[mesh_index]];

		float d = FLT_MAX;
		int triangle_index;
        if (!intersect_bvh(world, mesh.bvh_first, mesh.vi_first, ray, &triangle_index, &d)) {
            continue;
        }

		float3 N = world->normals.data[world->normal_indices.data[mesh.ni_first + 3 * triangle_index]].xyz;

		bool inside = false;

		// triangle normal is looking in wrong direction.
		if (dot(N, ray.dir) > 0) {
			inside = true;
			// N = -N;
			// without this noise will appear
			d -= tolerance;
		}

		if (d > tolerance && d < min_distance) {
			hit = true;
			*mat = mesh.material_index;
			min_distance = d;

			*pos = d * ray.dir + ray.origin;
			*normal = N;
			*hit_from_inside = inside;
		}
	}

	*normal = normalize(*normal);

	return hit;
}

// Schlick's approximation
float reflectance(float cosine, float ref_idx) {
	float r0 = (1-ref_idx) / (1+ref_idx);
	r0 = r0*r0;
	return r0 + (1-r0)*pow((1 - cosine),5);
}

float3 reflect(float3 v, float3 n) {
	return v - 2.0f * dot(v, n) * n;
}

float3 refract(float3 ray_dir, float3 normal, float etai_over_etat) {
    float cos_theta = fmin(dot(-ray_dir, normal), 1.0f);
    float3 r_out_perp =  etai_over_etat * (ray_dir + cos_theta*normal);
    float3 r_out_parallel = (float)-sqrt(fabs(1.0 - dot(r_out_perp, r_out_perp))) * normal;
    return r_out_perp + r_out_parallel;
}

float3 ray_bounce(World *world, Ray ray, int bounce_count) {
	float3 pos, normal;
	int mat_index;
	bool hit_from_inside;

    float3 color = 1.0f;
	for (int bounce = 0; bounce < bounce_count; ++bounce) {
		if (ray_cast(world, ray, &pos, &normal, &mat_index, &hit_from_inside)) {
			Material mat = world->materials.data[mat_index];

			ray.origin = pos;

			float cos_theta = -dot(normal, ray.dir); // normal and ray dir have length 1
			float cos_att = 1; 
			if (hit_from_inside) {
				if (mat.refractiveness == 0) {
					return 0;
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
                cos_att = 1.0f;
                bounce--;
			} else {
				cos_att = clamp(cos_theta, 0.0f, 1.0f);
				if (bounce == 0)  cos_att = 1;

				// update ray
				float3 reflection_ray = reflect(ray.dir, normal);
				ray.dir = mix(normalize(rand_vec()), reflection_ray, mat.specular);
				if (dot(ray.dir, normal) < 0) {
					ray.dir = -ray.dir;
				}
			}

            ray.invdir = 1.0f / ray.dir;

            if (mat.emissive) {
                return cos_att * VEC3(mat.color) * color;
            } else {
                color *= cos_att * VEC3(mat.color);
            }
		}
		else {
            if (world->materials.data[0].emissive) {
                return VEC3(world->materials.data[0].color);
            }
            return 0.0f;
		}
	}

	return 0.0f;
}

float4 ray_color(World *world, Ray ray, int sample_count, int bounce_count, int prev_count, float4 *sum) {
	for (int sample = 0; sample < sample_count; ++sample) {
		*sum += (float4)(ray_bounce(world, ray, bounce_count), 1.0f);
	}

	return *sum / (float)(sample_count + prev_count);
}
