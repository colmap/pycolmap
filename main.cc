#include <pybind11/pybind11.h>

namespace py = pybind11;

#include "absolute_pose.cc"
#include "essential_matrix.cc"
#include "fundamental_matrix.cc"
#include "transformations.cc"

PYBIND11_MODULE(pycolmap, m) {
    m.doc() = "COLMAP plugin";
    m.def("absolute_pose_estimation", &absolute_pose_estimation, "Absolute pose estimation with non-linear refinement.");
    m.def("essential_matrix_estimation", &essential_matrix_estimation, "LORANSAC + 5-point algorithm.");
    m.def("fundamental_matrix_estimation", &fundamental_matrix_estimation, "LORANSAC + 7-point algorithm.");
    m.def("image_to_world", &image_to_world, "Image to world transformation.");
    m.def("world_to_image", &world_to_image, "World to image transformation.");
}
