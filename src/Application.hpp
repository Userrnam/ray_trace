#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <vector>

#include "world.hpp"
#include "math.hpp"
#include "Camera.hpp"

struct Application {
	GLFWwindow *window = nullptr;
    World *_world = nullptr;
    Camera _camera;
    int samples = 5;

    // image size is equal to window size
    std::vector<vec3> _image;
    int _width, _height; 
    unsigned int VBO, VAO;
    unsigned int shader;
    unsigned int texture;

    bool init(int width, int height);
    void set_world(World *world);
    void run();

    void update_image();
    void setup_raytracing();
    void render_pass(int iteration, std::vector<vec3>& sums);

    // create VBO, VAO, Texture, Shaders
    void create_rendering_structures();
    void destroy_rendering_structures();

    void create_shader_program();
};
