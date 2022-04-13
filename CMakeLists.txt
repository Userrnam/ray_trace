cmake_minimum_required(VERSION 3.14)
project(ray_trace LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS " -O3")
include_directories(/usr/local/include)
add_executable(${PROJECT_NAME}
	src/stb_impl.cpp
	src/main.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE include)
