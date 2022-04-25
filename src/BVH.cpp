#include "BVH.hpp"

#include <float.h>
#include <algorithm>
#include <assert.h>

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
	// if ray is inside bounding box return true
	if (points[0].x < ray.origin.x && ray.origin.x < points[1].x &&
		points[0].y < ray.origin.y && ray.origin.y < points[1].y &&
		points[0].z < ray.origin.z && ray.origin.z < points[1].z) {
		return true;
	}

	// right
	if (ray.dir.x < -0.001) {
		float t = -(ray.origin.x - points[1].x) / ray.dir.x;

		vec3 p = ray.origin + t * ray.dir;
		if (t > 0 && points[0].y <= p.y && p.y <= points[1].y &&
			points[0].z <= p.z && p.z <= points[1].z) {
			return true;
		}
	}
	// back
	if (ray.dir.y < -0.001) {
		float t = -(ray.origin.y - points[1].y) / ray.dir.y;

		vec3 p = ray.origin + t * ray.dir;
		if (t > 0 && points[0].x <= p.x && p.x <= points[1].x &&
			points[0].z <= p.z && p.z <= points[1].z) {
			return true;
		}
	}
	// top
	if (ray.dir.z < -0.001) {
		float t = -(ray.origin.z - points[1].z) / ray.dir.z;

		vec3 p = ray.origin + t * ray.dir;
		if (t > 0 && points[0].x <= p.x && p.x <= points[1].x &&
			points[0].y <= p.y && p.y <= points[1].y) {
			return true;
		}
	}

	// left
	if (ray.dir.x > 0.001) {
		float t = -(ray.origin.x - points[0].x) / ray.dir.x;

		vec3 p = ray.origin + t * ray.dir;
		if (t > 0 && points[0].y <= p.y && p.y <= points[1].y &&
			points[0].z <= p.z && p.z <= points[1].z) {
			return true;
		}
	}
	// front
	if (ray.dir.y > 0.001) {
		float t = -(ray.origin.y - points[0].y) / ray.dir.y;

		vec3 p = ray.origin + t * ray.dir;
		if (t > 0 && points[0].x <= p.x && p.x <= points[1].x &&
			points[0].z <= p.z && p.z <= points[1].z) {
			return true;
		}
	}
	// bottom
	if (ray.dir.z > 0.001) {
		float t = -(ray.origin.z - points[0].z) / ray.dir.z;

		vec3 p = ray.origin + t * ray.dir;
		if (t > 0 && points[0].x <= p.x && p.x <= points[1].x &&
			points[0].y <= p.y && p.y <= points[1].y) {
			return true;
		}
	}
	return false;
}

bool BVH_Node::intersect(Ray ray, std::vector<int>& _vertex_indices, std::vector<int>& _normal_indices) const {
    if (!bounding_box.intersect(ray)) {
        return false;
    }
    // it's a leaf
    if (vertex_indices.size()) {
        for (int i : vertex_indices) {
            _vertex_indices.push_back(i);
        }
        for (int i : normal_indices) {
            _normal_indices.push_back(i);
        }
        return true;
    } else {
        if (!left || !right) {
            int k = 532;
        }

        assert(left);
        assert(right);

        bool l = left->intersect(ray, _vertex_indices, _normal_indices);
        bool r = right->intersect(ray, _vertex_indices, _normal_indices);

        return l || r;
    }
}

BVH_Node* BVH::build_recursive(Obj_File* obj_file, int split_axes, const std::vector<int>& vertex_indices, const std::vector<int>& normal_indices) {
    // allocate memory for root
    auto node = new BVH_Node;

    // calculate bounding volume.
	node->bounding_box.points[0] = obj_file->vertices[vertex_indices[0]];
	node->bounding_box.points[1] = obj_file->vertices[vertex_indices[0]];
	for (int index : vertex_indices) {
		auto& vertex = obj_file->vertices[index];
		for (int i = 0; i < 3; ++i) {
			if (vertex.arr()[i] > node->bounding_box.points[1].arr()[i]) {
				node->bounding_box.points[1].arr()[i] = vertex.arr()[i];
			}
			if (vertex.arr()[i] < node->bounding_box.points[0].arr()[i]) {
				node->bounding_box.points[0].arr()[i] = vertex.arr()[i];
			}
		}
	}

    if (vertex_indices.size() < 20) {
        node->vertex_indices = vertex_indices;
        node->normal_indices = normal_indices;
        return node;
    }

    std::vector<int> vertex_indices_copy = vertex_indices;
    std::sort(vertex_indices_copy.begin(), vertex_indices_copy.end(), [&](int a, int b) {
        return obj_file->vertices[a].x < obj_file->vertices[b].x;
        });

    float split_point = obj_file->vertices[vertex_indices_copy[vertex_indices_copy.size() / 2]].arr()[split_axes];
    std::vector<int> left_vertex_indices;
    std::vector<int> right_vertex_indices;
    std::vector<int> left_normal_indices;
    std::vector<int> right_normal_indices;

    for (int triangle_index = 0; triangle_index < vertex_indices.size(); triangle_index += 3) {
        // if all vertices of triangle are on the left side, add it to the left bounding box
        if (obj_file->vertices[vertex_indices[triangle_index + 0]].arr()[split_axes] < split_point &&
            obj_file->vertices[vertex_indices[triangle_index + 1]].arr()[split_axes] < split_point &&
            obj_file->vertices[vertex_indices[triangle_index + 2]].arr()[split_axes] < split_point
            ) {
            left_vertex_indices.push_back(vertex_indices[triangle_index + 0]);
            left_vertex_indices.push_back(vertex_indices[triangle_index + 1]);
            left_vertex_indices.push_back(vertex_indices[triangle_index + 2]);
            
            left_normal_indices.push_back(normal_indices[triangle_index + 0]);
            left_normal_indices.push_back(normal_indices[triangle_index + 1]);
            left_normal_indices.push_back(normal_indices[triangle_index + 2]);
        } else {
            right_vertex_indices.push_back(vertex_indices[triangle_index + 0]);
            right_vertex_indices.push_back(vertex_indices[triangle_index + 1]);
            right_vertex_indices.push_back(vertex_indices[triangle_index + 2]);
            
            right_normal_indices.push_back(normal_indices[triangle_index + 0]);
            right_normal_indices.push_back(normal_indices[triangle_index + 1]);
            right_normal_indices.push_back(normal_indices[triangle_index + 2]);
        }
    }

    // all indices on the left 
    if (left_vertex_indices.size() == 0) {
        node->vertex_indices = right_vertex_indices;
        node->normal_indices = right_normal_indices;
        return node;
    }
    // all indices on the right 
    if (right_vertex_indices.size() == 0) {
        node->vertex_indices = left_vertex_indices;
        node->normal_indices = left_normal_indices;
        return node;
    }

    split_axes = (split_axes + 1) % 3;

    node->left  = build_recursive(obj_file, split_axes, left_vertex_indices, left_normal_indices);
    node->right = build_recursive(obj_file, split_axes, right_vertex_indices, right_normal_indices);

    return node;
}

void BVH::build(Mesh* mesh, Obj_File* obj_file) {
    _root = build_recursive(obj_file, 0, mesh->vertex_indices, mesh->normal_indices);
}

void BVH::destroy_recursive(BVH_Node *node) {
    if (!node) {
        return;
    }
	destroy_recursive(node->left);
	destroy_recursive(node->right);
    delete node;
}

void BVH::destroy() {
    destroy_recursive(_root);
}

bool BVH::intersect(Ray ray, std::vector<int>& vertex_indices, std::vector<int>& normal_indices) const {
    return _root->intersect(ray, vertex_indices, normal_indices);
}

