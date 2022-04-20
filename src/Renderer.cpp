#include "Renderer.hpp"

#include "Ray_Trace.hpp"

void Renderer::set_world(World *world) {
    _world = world;
}

void Renderer::set_camera(Camera camera) {
    _camera_update_mutex.lock();
    _camera = camera;
    _restart = true;
    _camera_update_mutex.unlock();
}

void Renderer::run() {
    int iteration = 0;
    int _samples = 1;

    while (!_stop) {
        Ray ray;
        vec3 color;
        for (int32_t y = 0; y < _camera.get_height(); ++y) {
            for (int32_t x = 0; x < _camera.get_width(); ++x) {

                if (_restart) {
                    goto restart;
                }

                ray = _camera.get_ray(x, y);

                color = ray_color(_world, ray, _samples, 5, _samples * iteration, &_sums[y * _camera.get_width() + x]);

                _image[y * _camera.get_width() + x] = color;
            }
        }
        iteration++;
        // copy data from image to rendered_image
        _rendered_image_mutex.lock();
        memcpy(&_rendered_image[0], &_image[0], _image.size() * sizeof(_image[0]));
        _rendered_image_mutex.unlock();
        continue;
restart:
        _camera_update_mutex.lock();
        _rendered_image.clear();
        _rendered_image.resize(_camera.get_width() * _camera.get_height());
        _image.clear();
        _image.resize(_camera.get_width() * _camera.get_height());
        _sums.clear();
        _sums.resize(_camera.get_width() * _camera.get_height(), {});
        iteration = 0;
        _restart = false;
        _camera_update_mutex.unlock();
    }
}

void Renderer::stop() {
    _stop = true;
}

const std::vector<vec3> &Renderer::get_rendered_image() {
    _rendered_image_mutex.lock();
    return _rendered_image;
}

void Renderer::release_rendered_image() {
    _rendered_image_mutex.unlock();
}
