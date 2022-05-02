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

bool Bounding_Box::intersect(const Ray& ray) const {
    if (0) {
		// https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection
        //float _rd[4] = { ray.invdir.x, ray.invdir.y, ray.invdir.z, 0.0f };
        //float _ro[4] = { ray.origin.x, ray.origin.y, ray.origin.z, 0.0f };
        //float _p0[4] = { points[0].x, points[0].y, points[0].z, 0.0f };
        //float _p1[4] = { points[1].x, points[1].y, points[1].z, 0.0f };
        //__m128 rd = _mm_load_ps(_rd);
        //__m128 ro = _mm_load_ps(_ro);
        //__m128 p0 = _mm_load_ps(_p0);
        //__m128 p1 = _mm_load_ps(_p1);
        //__m128 p0_od = _mm_mul_ps(_mm_sub_ps(p0, ro), rd);
        //__m128 p1_od = _mm_mul_ps(_mm_sub_ps(p1, ro), rd);
        //__m128 zero = _mm_set1_ps(0.0f);
        //__m128 mask_lt = _mm_cmplt_ps(rd, zero);
        //__m128 mask_ge = _mm_cmpge_ps(rd, zero);
        //__m128 min = _mm_add_ps(_mm_and_ps(p1_od, mask_lt), _mm_and_ps(p0_od, mask_ge));
        //__m128 max = _mm_add_ps(_mm_and_ps(p0_od, mask_ge), _mm_and_ps(p0_od, mask_lt));

		float tmin =  (points[ray.dir.x <  0].x - ray.origin.x) * ray.invdir.x;
		float tmax =  (points[ray.dir.x >= 0].x - ray.origin.x) * ray.invdir.x;
		float tymin = (points[ray.dir.y <  0].y - ray.origin.y) * ray.invdir.y;
		float tymax = (points[ray.dir.y >= 0].y - ray.origin.y) * ray.invdir.y;

		if ((tmin > tymax) || (tymin > tmax))
			return false;

		if (tymin > tmin)
			tmin = tymin;
		if (tymax < tmax)
			tmax = tymax;

		float tzmin = (points[ray.dir.z < 0].z - ray.origin.z) * ray.invdir.z;
		float tzmax = (points[ray.dir.z >= 0].z - ray.origin.z) * ray.invdir.z;

		if ((tmin > tzmax) || (tzmin > tmax))
			return false;

		return true;
    }
    else {
        // if ray is inside bounding box return true
        if (points[0].x < ray.origin.x && ray.origin.x < points[1].x &&
            points[0].y < ray.origin.y && ray.origin.y < points[1].y &&
            points[0].z < ray.origin.z && ray.origin.z < points[1].z) {
            return true;
        }

        // right
        if (ray.dir.x < -0.001) {
            float t = (points[1].x - ray.origin.x) * ray.invdir.x;

            vec3 p = ray.origin + t * ray.dir;
            if (t > 0 && points[0].y <= p.y && p.y <= points[1].y &&
                points[0].z <= p.z && p.z <= points[1].z) {
                return true;
            }
        }
        // back
        if (ray.dir.y < -0.001) {
            float t = (points[1].y - ray.origin.y) * ray.invdir.y;

            vec3 p = ray.origin + t * ray.dir;
            if (t > 0 && points[0].x <= p.x && p.x <= points[1].x &&
                points[0].z <= p.z && p.z <= points[1].z) {
                return true;
            }
        }
        // top
        if (ray.dir.z < -0.001) {
            float t = (points[1].z - ray.origin.z) * ray.invdir.z;

            vec3 p = ray.origin + t * ray.dir;
            if (t > 0 && points[0].x <= p.x && p.x <= points[1].x &&
                points[0].y <= p.y && p.y <= points[1].y) {
                return true;
            }
        }

        // left
        if (ray.dir.x > 0.001) {
            float t = (points[0].x - ray.origin.x) * ray.invdir.x;

            vec3 p = ray.origin + t * ray.dir;
            if (t > 0 && points[0].y <= p.y && p.y <= points[1].y &&
                points[0].z <= p.z && p.z <= points[1].z) {
                return true;
            }
        }
        // front
        if (ray.dir.y > 0.001) {
            float t = (points[0].y - ray.origin.y) * ray.invdir.y;

            vec3 p = ray.origin + t * ray.dir;
            if (t > 0 && points[0].x <= p.x && p.x <= points[1].x &&
                points[0].z <= p.z && p.z <= points[1].z) {
                return true;
            }
        }
        // bottom
        if (ray.dir.z > 0.001) {
            float t = (points[0].z - ray.origin.z) * ray.invdir.z;

            vec3 p = ray.origin + t * ray.dir;
            if (t > 0 && points[0].x <= p.x && p.x <= points[1].x &&
                points[0].y <= p.y && p.y <= points[1].y) {
                return true;
            }
        }
        return false;
    }
}

bool BVH_Node::intersect(const std::vector<BVH_Node>& nodes, Ray ray, std::vector<int>& _triangle_indices) const {
    if (!bounding_box.intersect(ray)) {
        return false;
    }
    // it's a leaf
    if (triangle_indices.size()) {
        for (int i : triangle_indices) {
            _triangle_indices.push_back(i);
        }
        return true;
    } else {
        assert(left != -1);
        assert(right != -1);

        bool l = nodes[left].intersect(nodes, ray, _triangle_indices);
        bool r = nodes[right].intersect(nodes, ray, _triangle_indices);

        return l || r;
    }
}

BVH_Node BVH::build_recursive(Obj_File* obj_file, Mesh* mesh, int split_axes, const std::vector<int>& triangle_indices) {
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

    if (triangle_indices.size() < 10) {
        node.triangle_indices = triangle_indices;
        return node;
    }

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

    // all indices on the right 
    if (left_triangle_indices.size() == 0) {
        assert(right_triangle_indices.size() == triangle_indices.size());
        node.triangle_indices = right_triangle_indices;
        return node;
    }
    // all indices on the left 
    if (right_triangle_indices.size() == 0) {
        assert(left_triangle_indices.size() == triangle_indices.size());
        node.triangle_indices = left_triangle_indices;
        return node;
    }

    split_axes = (split_axes + 1) % 3;

    _nodes.push_back({});
    node.left = _nodes.size()-1;
    _nodes[node.left] = build_recursive(obj_file, mesh, split_axes, left_triangle_indices);

    _nodes.push_back({});
    node.right = _nodes.size()-1;
    _nodes[node.right] = build_recursive(obj_file, mesh, split_axes, right_triangle_indices);

    return node;
}

void BVH::build(Mesh* mesh, Obj_File* obj_file) {
    _nodes.push_back({});
    _root = _nodes.size()-1;
    std::vector<int> triangle_indices;
    triangle_indices.resize(mesh->vi_count / 3);
    for (int i = 0; i < triangle_indices.size(); ++i) {
        triangle_indices[i] = i;
    }
    _nodes[_root] = build_recursive(obj_file, mesh, 0, triangle_indices);
}

bool BVH::intersect(Ray ray, std::vector<int>& triangle_indices) const {
    return _nodes[_root].intersect(_nodes, ray, triangle_indices);
}

