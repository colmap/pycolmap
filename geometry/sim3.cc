#include <utility>

#include "colmap/geometry/sim3.h"
#include "colmap/geometry/rigid3.h"
#include "colmap/geometry/pose.h"

using namespace colmap;

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

namespace py = pybind11;


void init_sim3(py::module& m) {
    py::class_<Sim3d>(m, "Sim3d")
        .def(py::init<>())
        .def(py::init<double, const Eigen::Quaterniond&, const Eigen::Vector3d&>())
        .def_static("estimate", [](std::vector<Eigen::Vector3d> src,
                                   std::vector<Eigen::Vector3d> dst) {
                    Sim3d tgt_from_src;
                    bool success = tgt_from_src.Estimate(src, dst);
                    THROW_CHECK(success);
                    return tgt_from_src;
        })
        .def_readwrite("scale", &Sim3d::scale)
        .def_readwrite("rotation", &Sim3d::rotation)
        .def_readwrite("translation", &Sim3d::translation)
        .def_property_readonly("matrix", &Sim3d::ToMatrix)
        .def(py::self * Eigen::Vector3d())
        .def(py::self * Sim3d())
        .def("transform_camera_world", &TransformCameraWorld)
        .def("inverse", static_cast<Sim3d (*)(const Sim3d&)>(&Inverse))
        .def("__repr__", [](const Sim3d& self){
            std::stringstream ss;
            ss<<"Sim3d:\n"
            <<self.ToMatrix();
            return ss.str();
        });
}
