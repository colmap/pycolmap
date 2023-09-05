#include <utility>

#include "colmap/geometry/sim3.h"

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
        .def_property_readonly_static("estimate", [](py::object){
            return py::cpp_function([](std::vector<Eigen::Vector3d> src,
                    std::vector<Eigen::Vector3d> dst){
                Sim3d tform;
                bool success = tform.Estimate(src,dst);
                THROW_CHECK(success);
                return tform;
            });
        })
        .def_readwrite("scale", &Sim3d::scale)
        .def_readwrite("rotation", &Sim3d::rotation)
        .def_readwrite("translation", &Sim3d::translation)
        .def_property_readonly("matrix", &Sim3d::ToMatrix)
        .def(py::self * Eigen::Vector3d())
        .def(py::self * Sim3d())
        .def("transform_pose", [](const Sim3d& self,
                                  Eigen::Ref<Eigen::Vector4d> qvec,
                                  Eigen::Ref<Eigen::Vector3d> tvec)
                -> std::pair<Eigen::Vector4d, Eigen::Vector3d> {
            Eigen::Vector3d cpyt(tvec);
            Eigen::Vector4d cpyq(qvec);
            self.TransformPose(&cpyq, &cpyt);
            return {qvec, tvec};
        })
        .def("inverse", static_cast<Sim3d (*)(const Sim3d&)>(&Inverse))
        .def("__repr__", [](const Sim3d& self){
            std::stringstream ss;
            ss<<"Sim3d:\n"
            <<self.ToMatrix();
            return ss.str();
        });
}
