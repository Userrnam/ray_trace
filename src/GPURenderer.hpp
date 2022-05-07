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

    World* _world;

    struct arr {
        cl_mem data;
        int count;
    };

    struct {
        arr materials;

        arr spheres;

        arr mesh_indices;
        arr bvh_nodes;
        arr triangle_indices;

        arr vertex_indices;
        arr normal_indices;

        arr vertices;
        arr normals;

        arr meshes;
    } gpu_world;

    int _ray_bounce = 5;
    int _iteration = 0;

    bool _stop = false;
    int set_kernel_world_args(int start, cl_kernel kernel);
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

