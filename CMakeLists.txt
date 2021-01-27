project(3DCPPhysics)
cmake_minimum_required(VERSION 3.16)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS_DEBUG "-DENGINE_DEBUG")

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdeclspec -Weverything -Wno-unknown-argument -Wno-c++98-compat -Wno-c++17-extensions -Wno-c++98-compat-pedantic -Wno-global-constructors -Wno-exit-time-destructors -Wno-c99-extensions")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++2a")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-everything")
endif ()

add_library(
  3DCPPhysics SHARED
  ${CMAKE_CURRENT_LIST_DIR}/sources/PhysicsObject.cpp
  ${CMAKE_CURRENT_LIST_DIR}/sources/Shapes/AABB.cpp
  ${CMAKE_CURRENT_LIST_DIR}/sources/Shapes/OBB.cpp
  ${CMAKE_CURRENT_LIST_DIR}/sources/Shapes/Capsule.cpp
  ${CMAKE_CURRENT_LIST_DIR}/sources/Shapes/Raycasting.cpp
  ${CMAKE_CURRENT_LIST_DIR}/sources/Shapes/Sphere.cpp
  ${CMAKE_CURRENT_LIST_DIR}/sources/Maths/Quaternion.cpp
)

target_compile_definitions(
  3DCPPhysics PRIVATE

  SHARED_LIBRARY_EXPORT
)

target_include_directories(
    3DCPPhysics PRIVATE

  ${CMAKE_CURRENT_LIST_DIR}/sources
)

add_executable(
  example_OpenGL

  ${CMAKE_CURRENT_LIST_DIR}/examples/Camera.cpp
  ${CMAKE_CURRENT_LIST_DIR}/examples/OpenGL.cpp
)

target_include_directories(
  example_OpenGL PRIVATE

  ${CMAKE_CURRENT_LIST_DIR}/sources
  ${CMAKE_CURRENT_LIST_DIR}/examples
)

target_link_libraries(
  example_OpenGL PRIVATE

  GL
  glut
  GLU
  3DCPPhysics
)