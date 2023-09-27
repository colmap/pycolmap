cmake_minimum_required(VERSION 3.10)
project(PyCOLMAP)

set(CMAKE_CUDA_ARCHITECTURES "native")
find_package(colmap 3.9 REQUIRED)

if(${CERES_VERSION} VERSION_LESS "2.2.0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
endif()

add_subdirectory(pybind11)

pybind11_add_module(pycolmap main.cc)
target_include_directories(pycolmap PRIVATE ${PROJECT_SOURCE_DIR})
target_link_libraries(pycolmap PRIVATE colmap::colmap freeimage::FreeImage)
if(MSVC)
# these are added in COLMAP_LIBRARIES for UNIX systems but for some reasons they are omited for windows
# it crashes with LNK2019 because of that
# https://github.com/colmap/colmap/blob/9f3a75ae9c72188244f2403eb085e51ecf4397a8/cmake/CMakeConfig.cmake.in#L165
target_link_libraries(pycolmap PRIVATE ${Boost_FILESYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_SYSTEM_LIBRARY})
endif()
