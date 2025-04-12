cmake_minimum_required(VERSION 3.10)
project(3D_Visualization)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)

include_directories(${OPENGL_INCLUDE_DIR} ${GLEW_INCLUDE_DIRS})

add_executable(3d_viewer main.cpp)
target_link_libraries(3d_viewer ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} glfw)