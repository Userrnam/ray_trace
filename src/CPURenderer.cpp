#include "CPURenderer.hpp"

#include "Job_System.hpp"
#include "Ray_Trace.hpp"


CPURenderer::CPURenderer(int thread_count) {
    _thread_count = thread_count;
}

void CPURenderer::set_world(World *world) {
    _world = world;
    _restart = true;
}

void CPURenderer::set_camera(Camera camera) {
    _camera_update_mutex.lock();
    _camera = camera;
    _restart = true;
    _camera_update_mutex.unlock();
}

struct Global_Ray_Params {
    World* world;
    int samples;
    int ray_bounce;
    int iteration;
};

struct Ray_Params {
    Global_Ray_Params* global;
    vec3* color;
    Ray ray;
    vec3* sum;
};

void ray_color_job(void* user_data) {
    Ray_Params* params = (Ray_Params*)user_data;

	*params->color = ray_color(params->global->world, params->ray, params->global->samples,
        params->global->ray_bounce, params->global->samples * params->global->iteration, params->sum);
}

void CPURenderer::run() {
    Job_System job_system;

    job_system.create(_thread_count);

    Global_Ray_Params global;
    global.world = _world;
    global.iteration = _iteration;
    global.samples = 1;
    global.ray_bounce = _ray_bounce;

    std::vector<Ray_Params> params;
    params.resize(_camera.get_width() * _camera.get_height());

    _restart = true;

    int _samples = 1;
    while (!_stop) {
        Ray ray;
        vec3 color;
        for (int32_t y = 0; y < _camera.get_height(); ++y) {
            for (int32_t x = 0; x < _camera.get_width(); ++x) {

                if (_restart) {
					_camera_update_mutex.lock();

					// restart job system
					job_system.clear_jobs();
					job_system.wait_for_complition();
					global.world = _world;
					global.iteration = 0;
					params.resize(_camera.get_width() * _camera.get_height());

					// clear image
					_rendered_image.clear();
					_rendered_image.resize(_camera.get_width() * _camera.get_height());
					_image.clear();
					_image.resize(_camera.get_width() * _camera.get_height());
					_sums.clear();
					_sums.resize(_camera.get_width() * _camera.get_height(), {});
					_iteration = 0;
					_restart = false;

					_camera_update_mutex.unlock();
                }

                ray = _camera.get_ray(x, y);

                auto& ray_param = params[y * _camera.get_width() + x];
                ray_param.color = &_image[y * _camera.get_width() + x];
                ray_param.global = &global;
                ray_param.ray = ray;
                ray_param.sum = &_sums[y * _camera.get_width() + x];

                Job job;
                job.user_data = &ray_param;
                job.execute = ray_color_job;
                job_system.add_job(job);

                /*
                color = ray_color(_world, ray, _samples, ray_bounce, _samples * _iteration, &_sums[y * _camera.get_width() + x]);

                _image[y * _camera.get_width() + x] = color;
                */
            }
        }
        // execute available jobs with other threads
        while (job_system.execute_job()) {}
        _iteration++;
        global.iteration++;

        // copy data from image to rendered_image
        _rendered_image_mutex.lock();
		memcpy(&_rendered_image[0], &_image[0], _image.size() * sizeof(_image[0]));
        _rendered_image_mutex.unlock();
    }

    job_system.destroy();
}

void CPURenderer::stop() {
    _stop = true;
}

const std::vector<vec3> &CPURenderer::get_rendered_image() {
    _rendered_image_mutex.lock();
    return _rendered_image;
}

void CPURenderer::release_rendered_image() {
    _rendered_image_mutex.unlock();
}
