cmake_minimum_required(VERSION 3.14)
project(ray_trace LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS " -O3 -march=native")
add_link_options("-L/usr/local/lib")
add_link_options( -lglfw -framework OpenCL)
include_directories(/usr/local/include)
add_executable(${PROJECT_NAME}
	src/BVH.cpp
	src/stb_impl.cpp
	src/Obj_Loader.cpp
	src/Job_System.cpp
	src/Ray_Trace.cpp
	src/glad.cpp
	src/GPURenderer.cpp
	src/CPURenderer.cpp
	src/Application.cpp
	src/main.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE include)
