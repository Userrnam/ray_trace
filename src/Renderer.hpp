#pragma once

#include "World.hpp"
#include "Camera.hpp"

#include <mutex>

class Renderer {
    std::vector<vec3> _rendered_image;
    std::vector<vec3> _image; // this image is currently being rendered
    std::vector<vec3> _sums; // sum of all samples
    World *_world;
    Camera _camera;
    bool _stop = false;
    bool _restart = false;
    std::mutex _rendered_image_mutex;
    std::mutex _camera_update_mutex;
public:
    void set_world(World *world);

    // camera update will restart rendering
    void set_camera(Camera camera);

    void run();
    void stop();

    // returns rendered image, content of rendered image is not modified until the image is returned with release_rendered_image.
    const std::vector<vec3> &get_rendered_image();
    void release_rendered_image();
};
