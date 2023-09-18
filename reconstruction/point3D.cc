// Author: Philipp Lindenberger (Phil26AT)

#include "colmap/scene/point3d.h"

#include "colmap/util/misc.h"
#include "colmap/util/types.h"

using namespace colmap;

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;
using namespace pybind11::literals;

#include "log_exceptions.h"

PYBIND11_MAKE_OPAQUE(std::unordered_map<point3D_t, Point3D>);

std::string PrintPoint3D(const Point3D& point3D) {
  std::stringstream ss;
  ss << "<Point3D 'xyz=[" << point3D.XYZ().transpose()
     << "], track_length=" << (point3D.Track().Length())
     << ", error=" << point3D.Error() << "'>";
  return ss.str();
}

void init_point3D(py::module& m) {
  using Point3DMap = std::unordered_map<point3D_t, Point3D>;

  py::bind_map<Point3DMap>(m, "MapPoint3DIdPoint3D")
      .def("__repr__", [](const Point3DMap& self) {
        std::string repr = "{";
        bool is_first = true;
        for (auto& pair : self) {
          if (!is_first) {
            repr += ", ";
          }
          is_first = false;
          repr += std::to_string(pair.first) + ": " + PrintPoint3D(pair.second);
        }
        repr += "}";
        return repr;
      });

  py::class_<Point3D, std::shared_ptr<Point3D>>(m, "Point3D")
      .def(py::init<>())
      .def(py::init([](const Eigen::Vector3d& xyz, const Track& track) {
             std::unique_ptr<Point3D> point3D =
                 std::unique_ptr<Point3D>(new Point3D());
             point3D->SetXYZ(xyz);
             point3D->SetTrack(track);
             return point3D;
           }),
           "xyz"_a,
           "track"_a = Track())
      .def_property("xyz", py::overload_cast<>(&Point3D::XYZ), &Point3D::SetXYZ)
      .def_property("x",
                    &Point3D::X,
                    [](Point3D& self, double x) {
                      self.SetXYZ(Eigen::Vector3d(x, self.Y(), self.Z()));
                    })
      .def_property("y",
                    &Point3D::Y,
                    [](Point3D& self, double y) {
                      self.SetXYZ(Eigen::Vector3d(self.X(), y, self.Z()));
                    })
      .def_property("z",
                    &Point3D::Z,
                    [](Point3D& self, double z) {
                      self.SetXYZ(Eigen::Vector3d(self.X(), self.Y(), z));
                    })
      .def_property(
          "color", py::overload_cast<>(&Point3D::Color), &Point3D::SetColor)
      .def_property("error", &Point3D::Error, &Point3D::SetError)
      .def_property("track",
                    py::overload_cast<>(&Point3D::Track),
                    &Point3D::SetTrack,
                    py::return_value_policy::reference_internal)
      .def("__copy__", [](const Point3D& self) { return Point3D(self); })
      .def("__deepcopy__",
           [](const Point3D& self, py::dict) { return Point3D(self); })
      .def("__repr__", [](const Point3D& self) { return PrintPoint3D(self); })
      .def("summary", [](const Point3D& self) {
        std::stringstream ss;
        ss << "Point3D:\n\txyz = [" << self.XYZ().transpose()
           << "]\n\ttrack_length = " << (self.Track().Length())
           << "\n\terror = " << self.Error() << "\n\tcolor = ["
           << self.Color().cast<int>().transpose() << "]";
        return ss.str();
      });
}
