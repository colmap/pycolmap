#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

namespace py = pybind11;
using namespace pybind11::literals;

#include "log_exceptions.h"

template <typename... Args>
using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;


void init_quaternion(py::module& m) {
    py::class_<Eigen::Quaterniond>(m, "Rotation3d")
        .def(py::init<const Eigen::Vector4d&>(), "xyzw"_a)
        .def(py::init<const Eigen::Matrix3d&>(), "rotmat"_a)
        .def(py::self * Eigen::Quaterniond())
        .def(py::self * Eigen::Vector3d())
        .def("normalize", &Eigen::Quaterniond::normalize)
        .def("matrix", &Eigen::Quaterniond::toRotationMatrix)
        .def("quat", overload_cast_<>()(&Eigen::Quaterniond::coeffs))
        .def("norm", &Eigen::Quaterniond::norm)
        .def("inverse", &Eigen::Quaterniond::inverse)
        .def("__repr__", [](const Eigen::Quaterniond& self){
            std::stringstream ss;
            ss<<"Rotation3d: "
            <<self.coeffs();
            return ss.str();
        });
}
