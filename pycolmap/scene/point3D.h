// Author: Philipp Lindenberger (Phil26AT)

#include "colmap/scene/point3d.h"
#include "colmap/util/misc.h"
#include "colmap/util/types.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "pycolmap/helpers.h"
#include "pycolmap/log_exceptions.h"

using namespace colmap;
namespace py = pybind11;

using Point3DMap = std::unordered_map<point3D_t, Point3D>;
PYBIND11_MAKE_OPAQUE(Point3DMap);

void BindPoint3D(py::module& m) {
  py::bind_map_fix<Point3DMap>(m, "MapPoint3DIdPoint3D");

  auto PyPoint3D =
      py::class_<Point3D, std::shared_ptr<Point3D>>(m, "Point3D")
          .def(py::init<>())
          .def_readwrite("xyz", &Point3D::xyz)
          .def_readwrite("color", &Point3D::color)
          .def_readwrite("error", &Point3D::error)
          .def_readwrite("track", &Point3D::track)
          .def("__copy__", [](const Point3D& self) { return Point3D(self); })
          .def("__deepcopy__",
               [](const Point3D& self, py::dict) { return Point3D(self); });
  make_dataclass(PyPoint3D);
}
