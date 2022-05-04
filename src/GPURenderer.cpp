#include "GPURenderer.hpp"

#include <fstream>
#include <assert.h>
#include <GLFW/glfw3.h>


std::string read_file(const std::string& path) {
	std::fstream file;
	file.open(path, std::ios::in);

	// get file size
	file.seekg(0, std::ios::end);
	size_t size = file.tellg();
	file.seekg(0);

	std::string res(size, ' ');
	file.read(&res[0], size);

	file.close();

	return res;
}

GPURenderer::GPURenderer() {
	// get number of platforms
	cl_uint platform_count;
	cl_int ret = clGetPlatformIDs(0, NULL, &platform_count);

	std::vector<cl_platform_id> platforms(platform_count);
	ret = clGetPlatformIDs(platforms.size(), platforms.data(), NULL);

	cl_uint device_count;
	ret = clGetDeviceIDs(platforms[0], CL_DEVICE_TYPE_GPU, 1, &_device_id, &device_count);
	
	// Create an OpenCL context
	_context = clCreateContext(NULL, 1, &_device_id, NULL, NULL, &ret);

	// Create a command queue
	_command_queue = clCreateCommandQueue(_context, _device_id, 0, &ret);
}

GPURenderer::~GPURenderer() {
	/*
	
	clFlush(_command_queue);
	clFinish(_command_queue);

	if (_vertices)  clReleaseMemObject(_vertices);
	if (_normals)   clReleaseMemObject(_normals);
	if (_meshes)    clReleaseMemObject(_meshes);
	if (_materials) clReleaseMemObject(_materials);

	for (auto mesh : _meshes_vector) {
		clReleaseMemObject(mesh.bvh);
		clReleaseMemObject(mesh.normal_indices);
		clReleaseMemObject(mesh.vertex_indices);
	}

	clReleaseCommandQueue(_command_queue);
	clReleaseContext(_context);
	*/
}

void GPURenderer::set_world(World* world) {
	std::lock_guard<std::mutex> guard(_restart_mutex);
	_restart = true;

	_world = world;

#define CREATE_AND_WRITE(arr, vec) \
	if (arr.data) clReleaseMemObject(arr.data); \
	arr.data = clCreateBuffer(_context, CL_MEM_READ_ONLY, vec.size() * sizeof(vec[0]), NULL, &ret); assert(ret == 0); \
	arr.count = vec.size(); \
	ret = clEnqueueWriteBuffer(_command_queue, arr.data, CL_TRUE, 0, vec.size() * sizeof(vec[0]), \
		vec.data(), 0, NULL, NULL); assert(ret == 0)

	cl_int ret;
	CREATE_AND_WRITE(gpu_world.materials, world->materials);
	CREATE_AND_WRITE(gpu_world.mesh_indices, world->mesh_indices);
	CREATE_AND_WRITE(gpu_world.bvh_nodes, world->obj.bvh_nodes);
	CREATE_AND_WRITE(gpu_world.triangle_indices, world->obj.triangle_indices);

	CREATE_AND_WRITE(gpu_world.vertex_indices, world->obj.vertex_indices);
	CREATE_AND_WRITE(gpu_world.normal_indices, world->obj.normal_indices);

	CREATE_AND_WRITE(gpu_world.vertices, world->obj.vertices);
	CREATE_AND_WRITE(gpu_world.normals, world->obj.normals);

	CREATE_AND_WRITE(gpu_world.meshes, world->obj.meshes);
#undef CREATE_AND_WRITE
}

void GPURenderer::set_camera(Camera camera) {
	std::lock_guard<std::mutex> guard(_restart_mutex);
	_camera = camera;
	_restart = true;
}

int GPURenderer::set_kernel_world(int start, cl_kernel kernel) {
	cl_int ret;

#define SET_WORLD_PARAMETER(i, vec) \
	ret = clSetKernelArg(kernel, start + 2 * i, sizeof(cl_mem), &vec.data); assert(ret == 0); \
	ret = clSetKernelArg(kernel, start + 2 * i+1, sizeof(int), &vec.count); assert(ret == 0)

	SET_WORLD_PARAMETER(0, gpu_world.materials);
	SET_WORLD_PARAMETER(1, gpu_world.mesh_indices);
	SET_WORLD_PARAMETER(2, gpu_world.bvh_nodes);
	SET_WORLD_PARAMETER(3, gpu_world.triangle_indices);

	SET_WORLD_PARAMETER(4, gpu_world.vertex_indices);
	SET_WORLD_PARAMETER(5, gpu_world.normal_indices);

	SET_WORLD_PARAMETER(6, gpu_world.vertices);
	SET_WORLD_PARAMETER(7, gpu_world.normals);

	SET_WORLD_PARAMETER(8, gpu_world.meshes);
#undef SET_WORLD_PARAMETER

	return start + 2 * 9;
}

void GPURenderer::run() {
	_restart = false;

	_rendered_image.resize(_camera.get_width() * _camera.get_height());

	cl_int ret;
	cl_mem image = clCreateBuffer(_context, CL_MEM_WRITE_ONLY, _rendered_image.size() * 4 * sizeof(float), NULL, &ret);
	assert(ret == 0);

	cl_mem sums = clCreateBuffer(_context, CL_MEM_READ_WRITE, _rendered_image.size() * 4 * sizeof(float), NULL, &ret);
	assert(ret == 0);

	auto src_main = read_file("kernels/main.cl");
	char *data[] = {
		&src_main[0],
	};
	size_t sizes[] = {
		src_main.size(),
	};

	// create kernel
	cl_program program = clCreateProgramWithSource(_context, 1, (const char **)data, sizes, &ret);
	assert(ret == 0);
	ret = clBuildProgram(program, 1, &_device_id, "-I kernels", NULL, NULL);

	if (ret) {
		char log[512];
		size_t info_size = 0;
		ret = clGetProgramBuildInfo(program, _device_id, CL_PROGRAM_BUILD_LOG, 512, log, &info_size);
		assert(ret == 0);
		if (info_size) {
			std::cout << log << std::endl;
		}
		assert(0);
	}

	cl_kernel kernel = clCreateKernel(program, "main", &ret);
	assert(ret == 0);

	int arg_index = 0;
	ret = clSetKernelArg(kernel, arg_index++, sizeof(cl_mem), &image);

	ret = clSetKernelArg(kernel, arg_index++, sizeof(Camera), &_camera);
	assert(ret == 0);

	arg_index = set_kernel_world(arg_index, kernel);

	ret = clSetKernelArg(kernel, arg_index++, sizeof(int), &_ray_bounce);
	assert(ret == 0);

	int iteration_arg_index = arg_index;
	ret = clSetKernelArg(kernel, arg_index++, sizeof(int), &_iteration);
	assert(ret == 0);

	u32 xor_shift_32();
	u32 seed = xor_shift_32();
	ret = clSetKernelArg(kernel, arg_index++, sizeof(u32), &time);
	assert(ret == 0);

	ret = clSetKernelArg(kernel, arg_index++, sizeof(cl_mem), &sums);
	assert(ret == 0);

	// contents of GPU buffer is copied here due to alignment requirements.
	std::vector<float> tmp(4 * _rendered_image.size());

	while (!_stop) {
		while (_restart) {
			std::lock_guard<std::mutex> guard(_restart_mutex);
			_restart = false;

			_iteration = 0;

			if (_camera.get_width() * _camera.get_height() != _rendered_image.size()) {
				_rendered_image_mutex.lock();
				_rendered_image.resize(_camera.get_width() * _camera.get_height());
				_rendered_image_mutex.unlock();
				tmp.resize(4 * _rendered_image.size());

				// recreate image
				clReleaseMemObject(image);
				clReleaseMemObject(sums);
				image = clCreateBuffer(_context, CL_MEM_WRITE_ONLY, _rendered_image.size() * 4 * sizeof(float), NULL, &ret);
				assert(ret == 0);
				sums = clCreateBuffer(_context, CL_MEM_READ_WRITE, _rendered_image.size() * 4 * sizeof(float), NULL, &ret);
				assert(ret == 0);
			}

			// fill image with zeros.
			cl_event events[2];
			float pattern = 0;
			ret = clEnqueueFillBuffer(_command_queue, image, &pattern, sizeof(float), 0,
				_rendered_image.size() * 4 * sizeof(float), 0, NULL, &events[0]);
			assert(ret == 0);
			ret = clEnqueueFillBuffer(_command_queue, sums, &pattern, sizeof(float), 0,
				_rendered_image.size() * 4 * sizeof(float), 0, NULL, &events[1]);
			assert(ret == 0);
			ret = clWaitForEvents(2, events);
			assert(ret == 0);

			// update kernel args
			arg_index = 0;
			ret = clSetKernelArg(kernel, arg_index++, sizeof(cl_mem), &image);

			ret = clSetKernelArg(kernel, arg_index++, sizeof(Camera), &_camera);
			assert(ret == 0);

			arg_index = set_kernel_world(arg_index, kernel);

			ret = clSetKernelArg(kernel, arg_index++, sizeof(int), &_ray_bounce);
			assert(ret == 0);

			ret = clSetKernelArg(kernel, arg_index++, sizeof(int), &_iteration);
			assert(ret == 0);

			ret = clSetKernelArg(kernel, arg_index++, sizeof(float), &seed);
			assert(ret == 0);

			ret = clSetKernelArg(kernel, arg_index++, sizeof(cl_mem), &sums);
			assert(ret == 0);
		}

		// Execute the kernel
		size_t global_item_size = _camera.get_height();
		size_t local_item_size = 10;

		ret = clSetKernelArg(kernel, iteration_arg_index, sizeof(int), &_iteration);
		assert(ret == 0);

		seed = xor_shift_32();
		ret = clSetKernelArg(kernel, iteration_arg_index+1, sizeof(u32), &seed);
		assert(ret == 0);

		cl_event compute_finished;
		ret = clEnqueueNDRangeKernel(_command_queue, kernel, 1, NULL, &global_item_size, &local_item_size, 0, NULL, &compute_finished);
		assert(ret == 0);

		ret = clWaitForEvents(1, &compute_finished);
		assert(ret == 0);

		ret = clEnqueueReadBuffer(_command_queue, image, CL_TRUE, 0, _rendered_image.size() * 4 * sizeof(float),
			&tmp[0], 0, NULL, NULL);
		assert(ret == 0);

		_rendered_image_mutex.lock();
		for (int i = 0; i < _rendered_image.size(); ++i) {
			_rendered_image[i] = { tmp[4 * i + 0], tmp[4 * i + 1], tmp[4 * i + 2]};
		}

		_iteration++;
		_rendered_image_mutex.unlock();
	}

	clReleaseMemObject(image);
	clReleaseMemObject(sums);

	// remove kernel
	ret = clFlush(_command_queue);
	ret = clFinish(_command_queue);
	ret = clReleaseKernel(kernel);
	ret = clReleaseProgram(program);
}

void GPURenderer::stop() {
	_stop = true;
}

const std::vector<vec3>& GPURenderer::get_rendered_image() {
    _rendered_image_mutex.lock();
    return _rendered_image;
}

void GPURenderer::release_rendered_image() {
    _rendered_image_mutex.unlock();
}

