#pragma once

#include "World.hpp"
#include "Camera.hpp"


class Renderer {
public:
    virtual ~Renderer() {}

    virtual void set_world(World *world) = 0;
    virtual void set_ray_bounce(int ray_bounce) = 0; 
    virtual int get_iteration() = 0;

    // camera update will restart rendering
    virtual void set_camera(Camera camera) = 0;

    virtual void run() = 0;
    virtual void stop() = 0;

    // returns rendered image, content of rendered image is not modified until the image is returned with release_rendered_image.
    virtual const std::vector<vec3> &get_rendered_image() = 0;
    virtual void release_rendered_image() = 0;
};

