#include "Application.hpp"

#include <iostream>
#include <stb_image.h>

#include "Ray_Trace.hpp"

void error() {
    GLenum err;
    bool has_error = false;
    while((err = glGetError()) != GL_NO_ERROR) {
        std::cout << std::hex << int(err) << std::endl;
        has_error = true;
    }
    if (has_error) {
        exit(-1);
    }
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    auto app = (Application *)glfwGetWindowUserPointer(window);
    app->window_resized(width, height);
}

bool Application::init(int width, int height) {
    _width  = width;
    _height = height;

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    _window = glfwCreateWindow(width, height, "LearnOpenGL", NULL, NULL);

    if (_window == NULL) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(_window);
    glfwSetFramebufferSizeCallback(_window, framebuffer_size_callback);
    glfwSetWindowUserPointer(_window, this);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return false;
    }

    int w, h;
    glfwGetFramebufferSize(_window, &w, &h);
    glViewport(0, 0, w, h);

    create_rendering_structures();

    // create renderer
    // FIXME: move camera from here!
    _camera.create2(_width, _height, { 0, 15, 2 }, { 0, 1, 0 }, 1);
    _renderer = new Renderer;
    _renderer->set_camera(_camera);

    return true;
}

void Application::set_world(World *world) {
    _world = world;

    assert(_renderer);
    _renderer->set_world(_world);
}

void Application::run() {
    _renderer_thread = new std::thread(&Renderer::run, _renderer);

	while(!glfwWindowShouldClose(_window)) {
		glfwPollEvents();
        handle_input();

        update_image();

		glfwSwapBuffers(_window);
	}

    // stop renderer
    _renderer->stop();
    _renderer_thread->join();
    delete _renderer_thread;
    delete _renderer;

    destroy_rendering_structures();
	glfwTerminate();
}

void Application::handle_input() {
    // FIXME
    float velocity = 0.08;
    float velocity2 = 0.04;
    vec3 v = {};
    float r = 0;
    if (glfwGetKey(_window, GLFW_KEY_W) == GLFW_PRESS) v.y -= velocity;
    if (glfwGetKey(_window, GLFW_KEY_S) == GLFW_PRESS) v.y += velocity;
    if (glfwGetKey(_window, GLFW_KEY_A) == GLFW_PRESS) v.x -= velocity;
    if (glfwGetKey(_window, GLFW_KEY_D) == GLFW_PRESS) v.x += velocity;
    if (glfwGetKey(_window, GLFW_KEY_SPACE) == GLFW_PRESS) v.z += velocity;
    if (glfwGetKey(_window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) v.z -= velocity;
    if (glfwGetKey(_window, GLFW_KEY_LEFT) == GLFW_PRESS)  r += velocity2;
    if (glfwGetKey(_window, GLFW_KEY_RIGHT) == GLFW_PRESS) r -= velocity2;

    if (glfwGetKey(_window, GLFW_KEY_R) == GLFW_PRESS) {
        _renderer->set_camera(_camera);
    }

    bool updated = false;
    if (v.x != 0 || v.y != 0 || v.z != 0) {
        _camera.move(v);
        std::cout << "pos: " << _camera.get_position() << std::endl;
        std::cout << "dir: " << _camera.get_direction() << std::endl;
        updated = true;
    }
    if (r) {
        _camera.rotate_z(r);
        updated = true;
    }
    if (updated) {
        _renderer->set_camera(_camera);
    }
}

void Application::window_resized(int width, int height) {
    _width = width;
    _height = height;

    int w, h;
    glfwGetFramebufferSize(_window, &w, &h);
    glViewport(0, 0, w, h);

    // update width and size
	_camera.resize(_width, _height);
    _renderer->set_camera(_camera);
}

struct Color {
	u8 r;
	u8 g;
	u8 b;
	u8 a;
};

Color vec_to_color(vec3 v) {
	return Color {
		u8(255 * v.x),
		u8(255 * v.y),
		u8(255 * v.z),
		u8(255),
	};
}

void Application::update_image() {
    auto image = _renderer->get_rendered_image();

    // load texture
	std::vector<Color> buffer;
	buffer.resize(_width * _height, {});
    if (image.size() == buffer.size()) {
        for (int i = 0; i < image.size(); ++i) {
            buffer[i] = vec_to_color(clamp(image[i], vec3(0,0,0), vec3(1,1,1)));
        }
    }
    _renderer->release_rendered_image();

    // copy image to texture.
    glBindTexture(GL_TEXTURE_2D, _texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, _width, _height, 0, GL_RGBA, GL_UNSIGNED_BYTE, buffer.data());
    // for some reason it does not work without mipmaps
    glGenerateMipmap(GL_TEXTURE_2D);

    // draw image
    glBindTexture(GL_TEXTURE_2D, _texture);
    glUseProgram(_shader);
    glBindVertexArray(_VAO);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

    error();
}

void Application::create_rendering_structures() {
    const float vertices[] = {
        -1, -1,
         1, -1,
         1,  1,
        -1,  1
    };

    // create buffer and vertex array
    glGenBuffers(1, &_VBO);
    glGenVertexArrays(1, &_VAO);

    glBindVertexArray(_VAO);
        glBindBuffer(GL_ARRAY_BUFFER, _VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    create_shader_program();

    // create and bind texture
    glGenTextures(1, &_texture);
}

void Application::create_shader_program() {
    const char *vertex_src =
        "#version 330 core\n"
        "layout(location=0) in vec2 pos;\n"
        "out vec2 tex_coord;\n"
        "void main() {\n"
        "   gl_Position = vec4(pos.x, pos.y, 0.0, 1.0);\n"
        "   tex_coord = vec2(0.5 * (pos.x + 1.0), 0.5 * (pos.y + 1.0));\n" // we know it's a rect
        "}\n\0";
    
    const char *fragment_src =
        "#version 330 core\n"
        "uniform sampler2D image;\n"
        "in vec2 tex_coord;\n"
        "out vec4 FragColor;\n"
        "void main() {\n"
        "   FragColor = texture(image, tex_coord);\n"
        "}\n\0";

    unsigned int vertex_shader;
    vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &vertex_src, NULL);
    glCompileShader(vertex_shader);
    int  success;
    char infoLog[512];
    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &success);
    if(!success) {
        glGetShaderInfoLog(vertex_shader, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
    }

    unsigned int fragment_shader;
    fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &fragment_src, NULL);
    glCompileShader(fragment_shader);
    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &success);
    if(!success) {
        glGetShaderInfoLog(fragment_shader, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
    }

    _shader = glCreateProgram();
    glAttachShader(_shader, vertex_shader);
    glAttachShader(_shader, fragment_shader);
    glLinkProgram(_shader);
    glGetProgramiv(_shader, GL_LINK_STATUS, &success);
    if(!success) {
        glGetShaderInfoLog(_shader, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::PROGRAM::COMPILATION_FAILED\n" << infoLog << std::endl;
    }

    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);
}

void Application::destroy_rendering_structures() {
    glDeleteBuffers(1, &_VBO);
    glDeleteVertexArrays(1, &_VAO);
    glDeleteTextures(1, &_texture);
    glDeleteProgram(_shader);
}
