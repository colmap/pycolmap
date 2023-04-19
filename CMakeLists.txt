cmake_minimum_required(VERSION 3.6)
project(PyCOLMAP)

find_package(COLMAP REQUIRED)
include_directories(${COLMAP_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR})
link_directories(${COLMAP_LINK_DIRS})

if (${CERES_VERSION} VERSION_LESS "2.0.0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(${CERES_VERSION} VERSION_LESS "2.2.0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
endif()

add_subdirectory(pybind11)


if (${COLMAP_CUDA_ENABLED})
    message("Compiling bindings with CUDA support.")
    add_definitions(-DCUDA_ENABLED)
else()
    message("Compiling bindings without CUDA support.")
endif()

if (${COLMAP_CGAL_ENABLED})
    message("Compiling bindings with CGAL support.")
    add_definitions(-DCGAL_ENABLED)
else()
    message("Compiling bindings without CGAL support.")
endif()

pybind11_add_module(pycolmap main.cc)
target_link_libraries(pycolmap PRIVATE ${COLMAP_LIBRARIES})
if(MSVC)
# these are added in COLMAP_LIBRARIES for UNIX systems but for some reasons they are omited for windows
# it crashes with LNK2019 because of that
# https://github.com/colmap/colmap/blob/9f3a75ae9c72188244f2403eb085e51ecf4397a8/cmake/CMakeConfig.cmake.in#L165
target_link_libraries(pycolmap PRIVATE ${Boost_FILESYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_SYSTEM_LIBRARY})
endif()