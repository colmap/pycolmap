#include "colmap/geometry/rigid3.h"
#include "colmap/geometry/pose.h"

using namespace colmap;

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

namespace py = pybind11;


void init_rigid3(py::module& m) {
    py::class_<Rigid3d>(m, "Rigid3d")
        .def(py::init<>())
        .def(py::init<const Eigen::Quaterniond&, const Eigen::Vector3d&>())
        .def_readwrite("rotation", &Rigid3d::rotation)
        .def_readwrite("translation", &Rigid3d::translation)
        .def_property_readonly("matrix", &Rigid3d::ToMatrix)
        .def(py::self * Eigen::Vector3d())
        .def(py::self * Rigid3d())
        .def("inverse", static_cast<Rigid3d (*)(const Rigid3d&)>(&Inverse))
        .def_static("interpolate", &InterpolateCameraPoses)
        .def("__repr__", [](const Rigid3d& self){
            std::stringstream ss;
            ss<<"Rigid3d:\n"
            <<self.ToMatrix();
            return ss.str();
        });
}
