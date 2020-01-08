cmake_minimum_required(VERSION 3.6)
project(PyCOLMAP)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(COLMAP REQUIRED)
include_directories(${COLMAP_INCLUDE_DIRS})
link_directories(${COLMAP_LINK_DIRS})

find_package(pybind11 REQUIRED)
include_directories(${pybind11_INCLUDE_DIRS})

pybind11_add_module(pycolmap main.cc)
target_link_libraries(pycolmap PRIVATE ${COLMAP_LIBRARIES})