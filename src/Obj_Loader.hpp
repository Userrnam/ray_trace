#pragma once

#include <vector>
#include <unordered_map>
#include <string>

#include "Math.hpp"
#include "Bounding_Box.hpp"


// FIXME: move this from here.
struct BVH_Node {
    Bounding_Box bounding_box;
    BVH_Node* left  = nullptr;
    BVH_Node* right = nullptr;
    // if it's leaf
    std::vector<int> vertex_indices;
    std::vector<int> normal_indices;

    // returns vector of triangles to check.
    bool intersect(Ray ray, std::vector<int>& vertex_indices, std::vector<int>& normal_indices) const;
};

class BVH {
    BVH_Node* _root;

    BVH_Node* build_recursive(struct Obj_File* obj_file, int split_axes, const std::vector<int>& vertex_indices, const std::vector<int>& normal_indices);
    void destroy_recursive(BVH_Node *node);
public:
    void build(struct Mesh* mesh, struct Obj_File* obj_file);
    void destroy();

    // returns vector of triangles to check.
    bool intersect(Ray ray, std::vector<int>& vertex_indices, std::vector<int>& normal_indices) const;
};

struct Mesh {
    BVH bvh;
    Bounding_Box bounding_box;
    std::string name;
    std::vector<int> vertex_indices;
    std::vector<int> normal_indices;
};

struct Obj_File {
    std::vector<vec3> vertices;
    std::vector<vec3> normals;
    std::vector<Mesh> meshes;
    std::unordered_map<std::string, int> mesh_index;

    void load(const std::string& path);
};

