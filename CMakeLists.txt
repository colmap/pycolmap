cmake_minimum_required(VERSION 3.6)
project(PyCOLMAP)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS
             program_options
             filesystem
             graph
             regex
             system
             unit_test_framework)
find_package(COLMAP REQUIRED)
include_directories(${COLMAP_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${Boost_LINK_DIRS} ${COLMAP_LINK_DIRS})

add_subdirectory(pybind11)

pybind11_add_module(pycolmap main.cc)
target_link_libraries(pycolmap PRIVATE Boost::program_options Boost::filesystem Boost::graph Boost::regex Boost::system Boost::unit_test_framework ${COLMAP_LIBRARIES})