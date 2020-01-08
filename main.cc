#include <pybind11/pybind11.h>

namespace py = pybind11;

#include "absolute_pose.cc"
#include "essential_matrix.cc"

PYBIND11_MODULE(pycolmap, m) {
    m.doc() = "COLMAP plugin";
    m.def("absolute_pose_estimation", &absolute_pose_estimation, "Doc");
    m.def("essential_matrix_estimation", &essential_matrix_estimation, "Doc");
}
