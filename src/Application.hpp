#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <vector>
#include <thread>

#include "world.hpp"
#include "math.hpp"
#include "Camera.hpp"
#include "Renderer.hpp"


class Application {
public:
    bool init(int width, int height);
    void set_world(World *world);
    void run();

    // called by resize callback
    void window_resized(int width, int height);
private:
	GLFWwindow *_window = nullptr;
    World *_world = nullptr;
    Renderer *_renderer = nullptr;
    std::thread *_renderer_thread = nullptr;

    Camera _camera;
    int _width, _height; 
    unsigned int _VBO, _VAO;
    unsigned int _shader;
    unsigned int _texture;

    void update_image();
    void handle_input();

    // create VBO, VAO, Texture, Shaders
    void create_rendering_structures();
    void destroy_rendering_structures();

    void create_shader_program();
};
