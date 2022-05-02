#pragma once

#include "Renderer.hpp"

#include <mutex>
#include <CL/cl.h>

class GPURenderer : public Renderer {
    std::vector<vec3> _rendered_image;
    std::mutex _rendered_image_mutex;
    std::mutex _restart_mutex;
    Camera _camera;
    bool _restart = false;

    cl_device_id _device_id = nullptr;
    cl_context _context = nullptr;
    cl_command_queue _command_queue = nullptr;

    cl_mem _vertices = nullptr;
    cl_mem _normals = nullptr;
    cl_mem _indices = nullptr;
    cl_mem _materials = nullptr;

    struct GPUMesh {
        cl_mem bvh;
        cl_mem vertex_indices;
        cl_mem normal_indices;
    };

    cl_mem _meshes;
    std::vector<GPUMesh> _meshes_vector;

    int _ray_bounce = 0;
    int _iteration = 0;

    bool _stop = false;
public:
    GPURenderer();
    ~GPURenderer();

    void set_world(World *world) override;

    void set_ray_bounce(int ray_bounce) override { _ray_bounce = ray_bounce; }
    int get_iteration() override { return _iteration; }

    void set_camera(Camera camera) override;

    void run() override;
    void stop() override;

    // returns rendered image, content of rendered image is not modified until the image is returned with release_rendered_image.
    const std::vector<vec3> &get_rendered_image() override;
    void release_rendered_image() override;
};

