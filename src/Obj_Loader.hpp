#pragma once

#include <vector>
#include <unordered_map>
#include <string>

#include "Math.hpp"
#include "Bounding_Box.hpp"


struct Mesh {
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
