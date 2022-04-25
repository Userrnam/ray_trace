#include "Obj_Loader.hpp"

#include <fstream>
#include <sstream>


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

    for (auto& mesh : meshes) {
        mesh.bvh.build(&mesh, this);
    }
}

