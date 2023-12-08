cmake_minimum_required(VERSION 3.10)
project(${SKBUILD_PROJECT_NAME} VERSION ${SKBUILD_PROJECT_VERSION})

set(CMAKE_CUDA_ARCHITECTURES "native")
find_package(colmap 3.9 REQUIRED)

find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)

add_subdirectory(pybind11)

pybind11_add_module(pycolmap main.cc)
target_include_directories(pycolmap PRIVATE ${PROJECT_SOURCE_DIR})
target_link_libraries(pycolmap PRIVATE colmap::colmap freeimage::FreeImage)
target_compile_definitions(pycolmap PRIVATE VERSION_INFO="${PROJECT_VERSION}")
install(TARGETS pycolmap LIBRARY DESTINATION .)
