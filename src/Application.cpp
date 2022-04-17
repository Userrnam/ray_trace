#include "Application.hpp"

#include <iostream>
#include <stb_image.h>

#include "ray_trace.hpp"

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
    app->_width = width;
    app->_height = height;

    int w, h;
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);
}

bool Application::init(int width, int height) {
    _width  = width;
    _height = height;

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    window = glfwCreateWindow(width, height, "LearnOpenGL", NULL, NULL);

    if (window == NULL) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetWindowUserPointer(window, this);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return false;
    }

    int w, h;
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);

    create_rendering_structures();
    setup_raytracing();

    return true;
}

void Application::set_world(World *world) {
    _world = world;
}

void Application::run() {
    std::vector<vec3> sums;
    sums.resize(_width * _height, {});

    int iteration = 0;
	while(!glfwWindowShouldClose(window)) {
        render_pass(iteration++, sums);
        update_image();

		glfwSwapBuffers(window);
		glfwPollEvents();
	}
    destroy_rendering_structures();
	glfwTerminate();
}

void Application::setup_raytracing() {
    _image.resize(_width * _height);

    for (int i = 0; i < _width * _height; ++i) {
        _image[i] = vec3(1, 1, 1);
    }

	_camera.create(_width, _height, { 0, 10, 1}, { 3, 0, 0 }, 1);
}

void Application::render_pass(int iteration, std::vector<vec3>& sums) {
    int samples = 5;
	float max_color = {};
    for (int32_t y = 0; y < _height; ++y) {
        float v = ((float) y / (float) _height) * 2.0f - 1.0f;     // [-1; 1]
        for (int32_t x = 0; x < _width; ++x) {
            float u = ((float) x / (float) _width) * 2.0f - 1.0f;   // [-1; 1]

			Ray ray = _camera.get_ray(u, v);

			vec3 color = ray_color(_world, ray, samples, 3, samples * iteration, &sums[y * _width + x]);

			_image[y * _width + x] = color;
        }
    }
}

struct Color {
	u8 r;
	u8 g;
	u8 b;
};

Color vec_to_color(vec3 v) {
	return Color {
		u8(255 * v.x),
		u8(255 * v.y),
		u8(255 * v.z),
	};
}

void Application::update_image() {
    // load texture
	std::vector<Color> buffer;
	buffer.resize(_width * _height);
	for (int i = 0; i < _image.size(); ++i) {
		buffer[i] = vec_to_color(clamp(_image[i], vec3(0,0,0), vec3(1,1,1)));
	}

    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, _width, _height, 0, GL_RGB, GL_UNSIGNED_BYTE, buffer.data());
    // for some reason it does not work without mipmaps
    glGenerateMipmap(GL_TEXTURE_2D);

    // draw image
    glBindTexture(GL_TEXTURE_2D, texture);
    glUseProgram(shader);
    glBindVertexArray(VAO);
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
    glGenBuffers(1, &VBO);
    glGenVertexArrays(1, &VAO);

    glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    create_shader_program();

    // create and bind texture
    glGenTextures(1, &texture);
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

    // not checking for errors, because we know shaders are correct and don't use fancy features

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

    shader = glCreateProgram();
    glAttachShader(shader, vertex_shader);
    glAttachShader(shader, fragment_shader);
    glLinkProgram(shader);
    glGetProgramiv(shader, GL_LINK_STATUS, &success);
    if(!success) {
        glGetShaderInfoLog(shader, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::PROGRAM::COMPILATION_FAILED\n" << infoLog << std::endl;
    }

    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);
}

void Application::destroy_rendering_structures() {
    glDeleteBuffers(1, &VBO);
    glDeleteVertexArrays(1, &VAO);
    glDeleteTextures(1, &texture);
    glDeleteProgram(shader);
}
