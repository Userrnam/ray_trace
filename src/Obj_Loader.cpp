#include "Obj_Loader.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <assert.h>


void Obj_File::load(const std::string& path) {
    std::fstream file;
    file.open(path, std::ios::in);

    while (file) {
        std::string line;
        std::getline(file, line);
        std::stringstream ss(line);

        std::string word;
        ss >> word;
        if (word == "o") {
            // new mesh
            ss >> word;
            meshes.push_back({});
            meshes.back().name = word;
            mesh_index[word] = meshes.size() - 1;
        } else if (word == "v") {
            // vertex
            vec3 v;
            ss >> v.x >> v.z >> v.y;
            vertices.push_back(v);
        } else if (word == "vn") {
            // normal
            vec3 v;
            ss >> v.x >> v.z >> v.y;
            normals.push_back(v);
        } else if (word == "f") {
            int vertex_indices[3];
            int normal_indices[3];
            for (int k = 0; k < 3; ++k) {
                std::string vertex;
                ss >> vertex;
                std::stringstream vertexss(vertex);
                int values[3];
                for (int i = 0; i < 3; ++i) {
                    std::getline(vertexss, word, '/');
                    if (word.size()) {
                        values[i] = std::stoi(word);
                    } else {
                        values[i] = -1;
                    }
                }
                meshes.back().vertex_indices.push_back(values[0]-1);
                if (values[2] != -1) {
                    meshes.back().normal_indices.push_back(values[2]-1);
                }
            }
        }
    }

    // calculate bounding boxes for meshes
    for (auto& mesh : meshes) {
        mesh.bounding_box.points[0] = vertices[mesh.vertex_indices[0]];
        mesh.bounding_box.points[1] = vertices[mesh.vertex_indices[0]];
        for (int index : mesh.vertex_indices) {
            auto& vertex = vertices[index];
            for (int i = 0; i < 3; ++i) {
				if (vertex.arr()[i] > mesh.bounding_box.points[1].arr()[i]) {
					mesh.bounding_box.points[1].arr()[i] = vertex.arr()[i];
				}
				if (vertex.arr()[i] < mesh.bounding_box.points[0].arr()[i]) {
					mesh.bounding_box.points[0].arr()[i] = vertex.arr()[i];
				}
            }
        }
        mesh.bvh.build(&mesh, this);
        int k = 52;
    }
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

