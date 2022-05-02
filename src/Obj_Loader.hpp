#pragma once

#include <vector>
#include <unordered_map>
#include <string>

#include "Math.hpp"
#include "BVH.hpp"


struct Mesh {
    BVH bvh;
    std::string name;
    int vi_first = 0; int vi_count = 0; // vertex indices
    int ni_first = 0; int ni_count = 0; // normal indices
    int material_index = 0;
};

// contains everything about meshes in file. Meshes do not store anything, they just refference data in this structure.
struct Obj_File {
    // refferenced by BVH
    std::vector<BVH_Node> bvh_nodes;
    std::vector<int> triangle_indices;

    // refferenced by Mesh
    std::vector<int> vertex_indices;
    std::vector<int> normal_indices;

    std::vector<vec3> vertices;
    std::vector<vec3> normals;
    std::vector<Mesh> meshes;
    std::unordered_map<std::string, int> mesh_index;

    void load(const std::string& path);
};

