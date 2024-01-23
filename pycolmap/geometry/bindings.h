#include "colmap/geometry/pose.h"
#include "colmap/geometry/rigid3.h"
#include "colmap/geometry/sim3.h"

#include "pycolmap/geometry/homography_matrix.h"
#include "pycolmap/helpers.h"

#include <sstream>

#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;
using namespace pybind11::literals;

void BindGeometry(py::module& m) {
  BindHomographyGeometry(m);

  py::class_<Eigen::Quaterniond> PyRotation3d(m, "Rotation3d");
  PyRotation3d.def(py::init([]() { return Eigen::Quaterniond::Identity(); }))
      .def(py::init<const Eigen::Vector4d&>(), "xyzw"_a)
      .def(py::init<const Eigen::Matrix3d&>(), "rotmat"_a)
      .def(py::self * Eigen::Quaterniond())
      .def(py::self * Eigen::Vector3d())
      .def_property("quat",
                    py::overload_cast<>(&Eigen::Quaterniond::coeffs),
                    [](Eigen::Quaterniond& self, const Eigen::Vector4d& quat) {
                      self.coeffs() = quat;
                    })
      .def("normalize", &Eigen::Quaterniond::normalize)
      .def("matrix", &Eigen::Quaterniond::toRotationMatrix)
      .def("norm", &Eigen::Quaterniond::norm)
      .def("inverse", &Eigen::Quaterniond::inverse)
      .def("__repr__", [](const Eigen::Quaterniond& self) {
        std::stringstream ss;
        ss << "Rotation3d(quat_xyzw=[" << self.coeffs().format(vec_fmt) << "])";
        return ss.str();
      });
  py::implicitly_convertible<py::array, Eigen::Quaterniond>();
  MakeDataclass(PyRotation3d);

  py::class_<Rigid3d> PyRigid3d(m, "Rigid3d");
  PyRigid3d.def(py::init<>())
      .def(py::init<const Eigen::Quaterniond&, const Eigen::Vector3d&>())
      .def(py::init([](const Eigen::Matrix3x4d& matrix) {
        return Rigid3d(Eigen::Quaterniond(matrix.leftCols<3>()), matrix.col(3));
      }))
      .def_readwrite("rotation", &Rigid3d::rotation)
      .def_readwrite("translation", &Rigid3d::translation)
      .def("matrix", &Rigid3d::ToMatrix)
      .def(py::self * Eigen::Vector3d())
      .def(py::self * Rigid3d())
      .def("inverse", static_cast<Rigid3d (*)(const Rigid3d&)>(&Inverse))
      .def_static("interpolate", &InterpolateCameraPoses)
      .def("__repr__", [](const Rigid3d& self) {
        std::stringstream ss;
        ss << "Rigid3d("
           << "quat_xyzw=[" << self.rotation.coeffs().format(vec_fmt) << "], "
           << "t=[" << self.translation.format(vec_fmt) << "])";
        return ss.str();
      });
  py::implicitly_convertible<py::array, Rigid3d>();
  MakeDataclass(PyRigid3d);

  py::class_<Sim3d> PySim3d(m, "Sim3d");
  PySim3d.def(py::init<>())
      .def(
          py::init<double, const Eigen::Quaterniond&, const Eigen::Vector3d&>())
      .def(py::init(&Sim3d::FromMatrix))
      .def_readwrite("scale", &Sim3d::scale)
      .def_readwrite("rotation", &Sim3d::rotation)
      .def_readwrite("translation", &Sim3d::translation)
      .def("matrix", &Sim3d::ToMatrix)
      .def(py::self * Eigen::Vector3d())
      .def(py::self * Sim3d())
      .def("transform_camera_world", &TransformCameraWorld)
      .def("inverse", static_cast<Sim3d (*)(const Sim3d&)>(&Inverse))
      .def("__repr__", [](const Sim3d& self) {
        std::stringstream ss;
        ss << "Sim3d("
           << "scale=" << self.scale << ", "
           << "quat_xyzw=[" << self.rotation.coeffs().format(vec_fmt) << "], "
           << "t=[" << self.translation.format(vec_fmt) << "])";
        return ss.str();
      });
  py::implicitly_convertible<py::array, Sim3d>();
  MakeDataclass(PySim3d);
}
