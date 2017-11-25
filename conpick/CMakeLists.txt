cmake_minimum_required(VERSION 2.6)
project(conpick)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(PROJECT_MAJOR_VERSION 0)
set(PROJECT_MINOR_VERSION 0)
set(PROJECT_PATCH_VERSION 0)
set(PROJECT_VERSION
  "${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION}.${PROJECT_PATCH_VERSION}")

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLFW REQUIRED)

include_directories(${GLEW_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS})

add_subdirectory(Box2D)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Box2D)
add_subdirectory(sim)
