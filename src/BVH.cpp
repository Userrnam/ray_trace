#include "BVH.hpp"

#include <float.h>
#include <algorithm>
#include <assert.h>
//#include <emmintrin.h>
//#include <xmmintrin.h>

#include "Obj_Loader.hpp"


float intersect_p(const Ray& ray, vec3 normal, float d) {
	float denom = dot(normal, ray.dir);
	// ignore if plane normal is pointing in wrong direction
//	if (denom < -tolerance || denom > tolerance) {
	if (denom < -0.001) {
		return (-dot(normal, ray.origin) - d) / denom;
	}
	return -1;
}

#define INSIDE -10.0f
float Bounding_Box::intersect(const Ray& ray) const {
	// if ray is inside bounding box return true
	if (points[0].x < ray.origin.x && ray.origin.x < points[1].x &&
		points[0].y < ray.origin.y && ray.origin.y < points[1].y &&
		points[0].z < ray.origin.z && ray.origin.z < points[1].z) {
		return INSIDE;
	}

	// right
	if (ray.dir.x < -0.001) {
		float t = (points[1].x - ray.origin.x) * ray.invdir.x;

		vec3 p = ray.origin + t * ray.dir;
		if (t > 0 && points[0].y <= p.y && p.y <= points[1].y &&
			points[0].z <= p.z && p.z <= points[1].z) {
			return t;
		}
	}
	// back
	if (ray.dir.y < -0.001) {
		float t = (points[1].y - ray.origin.y) * ray.invdir.y;

		vec3 p = ray.origin + t * ray.dir;
		if (t > 0 && points[0].x <= p.x && p.x <= points[1].x &&
			points[0].z <= p.z && p.z <= points[1].z) {
			return t;
		}
	}
	// top
	if (ray.dir.z < -0.001) {
		float t = (points[1].z - ray.origin.z) * ray.invdir.z;

		vec3 p = ray.origin + t * ray.dir;
		if (t > 0 && points[0].x <= p.x && p.x <= points[1].x &&
			points[0].y <= p.y && p.y <= points[1].y) {
			return t;
		}
	}

	// left
	if (ray.dir.x > 0.001) {
		float t = (points[0].x - ray.origin.x) * ray.invdir.x;

		vec3 p = ray.origin + t * ray.dir;
		if (t > 0 && points[0].y <= p.y && p.y <= points[1].y &&
			points[0].z <= p.z && p.z <= points[1].z) {
			return t;
		}
	}
	// front
	if (ray.dir.y > 0.001) {
		float t = (points[0].y - ray.origin.y) * ray.invdir.y;

		vec3 p = ray.origin + t * ray.dir;
		if (t > 0 && points[0].x <= p.x && p.x <= points[1].x &&
			points[0].z <= p.z && p.z <= points[1].z) {
			return t;
		}
	}
	// bottom
	if (ray.dir.z > 0.001) {
		float t = (points[0].z - ray.origin.z) * ray.invdir.z;

		vec3 p = ray.origin + t * ray.dir;
		if (t > 0 && points[0].x <= p.x && p.x <= points[1].x &&
			points[0].y <= p.y && p.y <= points[1].y) {
			return t;
		}
	}
	return -1.0f;
}

static float tolerance = 0.0001f;
static float intersect_triangle(const Ray& ray, const vec3& v0, const vec3& v1, const vec3& v2) {
    vec3 e1 = v1 - v0;
    vec3 e2 = v2 - v0;

    // calculate triple product of e1, e2 and ray.dir
    vec3 pvec = cross(ray.dir, e2);
    float det = dot(e1, pvec);

    // ray and plane are parallel
    if (-tolerance < det && det < tolerance) {
        return -1;
    }

    float inv_det = 1 / det;
    vec3 tvec = ray.origin - v0;
    float u = dot(tvec, pvec) * inv_det;
    if (u < 0 || u > 1) {
        return -1;
    }

    vec3 qvec = cross(tvec, e1);
    float v = dot(ray.dir, qvec) * inv_det;
    if (v < 0 || u + v > 1) {
        return -1;
    }
    return dot(e2, qvec) * inv_det;
}

bool BVH_Node::intersect(const Obj_File* obj_file, int vi_first, Ray ray, int *triangle_index, float *t) const {
    if (bounding_box.intersect(ray) == -1.0f) {
        return false;
    }
    // it's a leaf
    if (triangle_count) {
        bool intersected = false;
        for (int i = 0; i < triangle_count; ++i) {
            int index = obj_file->triangle_indices[first_triangle + i];
			float distance = intersect_triangle(ray,
				obj_file->vertices[obj_file->vertex_indices[vi_first + 3 * index + 0]],
				obj_file->vertices[obj_file->vertex_indices[vi_first + 3 * index + 1]],
				obj_file->vertices[obj_file->vertex_indices[vi_first + 3 * index + 2]]
			);
            if (distance > tolerance && distance < *t) {
                intersected = true;
                *t = distance;
                *triangle_index = index;
            }
        }
        return intersected;
    } else {
        assert(left != -1);
        assert(right != -1);

        bool l = obj_file->bvh_nodes[left].intersect(obj_file,  vi_first, ray, triangle_index, t);
        bool r = obj_file->bvh_nodes[right].intersect(obj_file, vi_first, ray, triangle_index, t);

        return l || r;
    }
}

int choose_split_axes(struct Obj_File* obj_file, struct Mesh* mesh, const std::vector<int>& triangle_indices) {
	int split_axes = 0;
	float best_ratio = 10000;
    for (int axes = 0; axes < 3; ++axes) {
		std::vector<int> triangle_indices_copy = triangle_indices;
		std::sort(triangle_indices_copy.begin(), triangle_indices_copy.end(), [&](int a, int b) {
			auto va = obj_file->vertices[obj_file->vertex_indices[mesh->vi_first + 3 * a]];
			auto vb = obj_file->vertices[obj_file->vertex_indices[mesh->vi_first + 3 * b]];
			return va.arr()[axes] < vb.arr()[axes];
			});

		int split_index = triangle_indices_copy[triangle_indices_copy.size() / 2];
		float split_point = obj_file->vertices[obj_file->vertex_indices[mesh->vi_first + 3 * split_index]].arr()[axes];

		int left = 0;
		int right = 0;

		for (int triangle_index : triangle_indices) {
			// if all vertices of triangle are on the left side, add it to the left bounding box
			if (obj_file->vertices[obj_file->vertex_indices[mesh->vi_first + 3 * triangle_index + 0]].arr()[axes] < split_point &&
				obj_file->vertices[obj_file->vertex_indices[mesh->vi_first + 3 * triangle_index + 1]].arr()[axes] < split_point &&
				obj_file->vertices[obj_file->vertex_indices[mesh->vi_first + 3 * triangle_index + 2]].arr()[axes] < split_point
				) {
				left++;
			} else {
				right++;
			}
		}

		float ratio = float(left) / float(right);
		if (ratio < 1.0f) {
			ratio = float(right) / float(left);
		}
		if (ratio < best_ratio) {
			split_axes = axes;
			best_ratio = ratio;
		}
    }
	return split_axes;
}

BVH_Node BVH::build_recursive(Obj_File* obj_file, Mesh* mesh, const std::vector<int>& triangle_indices) {
    BVH_Node node = {};

    // calculate bounding volume.
	node.bounding_box.points[0] = obj_file->vertices[obj_file->vertex_indices[mesh->vi_first + triangle_indices[0]]];
	node.bounding_box.points[1] = obj_file->vertices[obj_file->vertex_indices[mesh->vi_first + triangle_indices[0]]];
	for (int triangle_index : triangle_indices) {
        for (int k = 0; k < 3; ++k) {
			auto& vertex = obj_file->vertices[obj_file->vertex_indices[mesh->vi_first + 3 * triangle_index + k]];
			for (int i = 0; i < 3; ++i) {
				if (vertex.arr()[i] > node.bounding_box.points[1].arr()[i]) {
					node.bounding_box.points[1].arr()[i] = vertex.arr()[i];
				}
				if (vertex.arr()[i] < node.bounding_box.points[0].arr()[i]) {
					node.bounding_box.points[0].arr()[i] = vertex.arr()[i];
				}
			}
        }
	}

    if (triangle_indices.size() < 18) {
        node.first_triangle = obj_file->triangle_indices.size();
        node.triangle_count = triangle_indices.size();
        for (int triangle_index : triangle_indices) {
            obj_file->triangle_indices.push_back(triangle_index);
        }
        return node;
    }

	int split_axes = choose_split_axes(obj_file, mesh, triangle_indices);

    std::vector<int> triangle_indices_copy = triangle_indices;
    std::sort(triangle_indices_copy.begin(), triangle_indices_copy.end(), [&](int a, int b) {
        auto va = obj_file->vertices[obj_file->vertex_indices[mesh->vi_first + 3 * a]];
        auto vb = obj_file->vertices[obj_file->vertex_indices[mesh->vi_first + 3 * b]];
        return va.arr()[split_axes] < vb.arr()[split_axes];
        });

    int split_index = triangle_indices_copy[triangle_indices_copy.size() / 2];
    float split_point = obj_file->vertices[obj_file->vertex_indices[mesh->vi_first + 3 * split_index]].arr()[split_axes];

    std::vector<int> left_triangle_indices;
    std::vector<int> right_triangle_indices;

    for (int triangle_index : triangle_indices) {
        // if all vertices of triangle are on the left side, add it to the left bounding box
        if (obj_file->vertices[obj_file->vertex_indices[mesh->vi_first + 3 * triangle_index + 0]].arr()[split_axes] < split_point &&
            obj_file->vertices[obj_file->vertex_indices[mesh->vi_first + 3 * triangle_index + 1]].arr()[split_axes] < split_point &&
            obj_file->vertices[obj_file->vertex_indices[mesh->vi_first + 3 * triangle_index + 2]].arr()[split_axes] < split_point
            ) {
            left_triangle_indices.push_back(triangle_index);
        } else {
            right_triangle_indices.push_back(triangle_index);
        }
    }

    // all indices on one side
    if (left_triangle_indices.size() == 0 || right_triangle_indices.size() == 0) {
        node.first_triangle = obj_file->triangle_indices.size();
        node.triangle_count = triangle_indices.size();
        for (int triangle_index : triangle_indices) {
            obj_file->triangle_indices.push_back(triangle_index);
        }
        return node;
    }

    split_axes = (split_axes + 1) % 3;

    obj_file->bvh_nodes.push_back({});
    node.left = obj_file->bvh_nodes.size()-1;
    obj_file->bvh_nodes[node.left] = build_recursive(obj_file, mesh, left_triangle_indices);

    obj_file->bvh_nodes.push_back({});
    node.right = obj_file->bvh_nodes.size()-1;
    obj_file->bvh_nodes[node.right] = build_recursive(obj_file, mesh, right_triangle_indices);

    return node;
}

void BVH::build(Mesh* mesh, Obj_File* obj_file) {
    _first = obj_file->bvh_nodes.size();
    obj_file->bvh_nodes.push_back({});
    std::vector<int> triangle_indices;
    triangle_indices.resize(mesh->vi_count / 3);
    for (int i = 0; i < triangle_indices.size(); ++i) {
        triangle_indices[i] = i;
    }
    obj_file->bvh_nodes[_first] = build_recursive(obj_file, mesh, triangle_indices);
}

bool BVH::intersect(const Obj_File* obj_file, int vi_start, Ray ray, int *triangle_index, float *t) const {
    *t = FLT_MAX;
    //return intersect_iterative(obj_file, vi_start, ray, triangle_index, t);
    return obj_file->bvh_nodes[_first].intersect(obj_file, vi_start, ray, triangle_index, t);
}

bool BVH::intersect_iterative(const Obj_File* obj_file, int vi_first, Ray ray, int *triangle_index, float *t) const {
    std::vector<int> stack;
    stack.push_back(_first);

	static int max_stack_size = 0;

	bool intersected = false;
    while (stack.size()) {
		if (stack.size() > max_stack_size) {
			max_stack_size = stack.size();
			std::cout << "Stack size: " << max_stack_size << std::endl;
		}

		auto node = obj_file->bvh_nodes[stack.back()];
        stack.pop_back();
		if (node.bounding_box.intersect(ray) == -1.0f) {
			continue;
		}
		// it's a leaf
		if (node.triangle_count) {
			for (int i = 0; i < node.triangle_count; ++i) {
				int index = obj_file->triangle_indices[node.first_triangle + i];
				float distance = intersect_triangle(ray,
					obj_file->vertices[obj_file->vertex_indices[vi_first + 3 * index + 0]],
					obj_file->vertices[obj_file->vertex_indices[vi_first + 3 * index + 1]],
					obj_file->vertices[obj_file->vertex_indices[vi_first + 3 * index + 2]]
				);
				if (distance > 0 && distance < *t) {
					intersected = true;
					*t = distance;
					*triangle_index = index;
				}
			}
		} else {
			assert(node.left != -1);
			assert(node.right != -1);

			stack.push_back(node.left);
			stack.push_back(node.right);
		}
    }
    return intersected;
}

