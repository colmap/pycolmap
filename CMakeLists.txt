cmake_minimum_required(VERSION 3.6)
project(PyCOLMAP)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(COLMAP REQUIRED)
include_directories(${COLMAP_INCLUDE_DIRS})
link_directories(${COLMAP_LINK_DIRS})

add_subdirectory(pybind11)

pybind11_add_module(pycolmap main.cc)
target_link_libraries(pycolmap PRIVATE ${COLMAP_LIBRARIES})
if(MSVC)
# these are added in COLMAP_LIBRARIES for UNIX systems but for some reasons they are omited for windows
# it crashes with LNK2019 because of that
# https://github.com/colmap/colmap/blob/9f3a75ae9c72188244f2403eb085e51ecf4397a8/cmake/CMakeConfig.cmake.in#L165
target_link_libraries(pycolmap PRIVATE ${Boost_FILESYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_SYSTEM_LIBRARY})
endif()