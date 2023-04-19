// Author: Philipp Lindenberger (Phil26AT)

// Use Eigens aligned allocator for vectors
#include<Eigen/StdVector>

#include "colmap/base/point2d.h"
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

template <typename... Args>
using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;

using vector_Point2D = std::vector<class colmap::Point2D, Eigen::aligned_allocator<colmap::Point2D>>;
PYBIND11_MAKE_OPAQUE(vector_Point2D);

std::string PrintPoint2D(const colmap::Point2D& p2D) {
    std::stringstream ss;
    ss << "<Point2D 'xy=[" << p2D.XY().transpose()
       << "], point3D_id=" << (p2D.HasPoint3D() ? std::to_string(p2D.Point3DId()) : "Invalid")
       << "'>";
    return ss.str();
}

void init_point2D(py::module& m) {
    py::bind_vector<vector_Point2D>(m, "ListPoint2D")
        .def("__repr__", [](const vector_Point2D& self) {
            std::string repr = "[";
            bool is_first = true;
            for (auto& p2D : self) {
                if (!is_first) {
                    repr += ", ";
                }
                is_first = false;
                repr += PrintPoint2D(p2D);
            }
            repr += "]";
            return repr;
        });

    py::class_<colmap::Point2D, std::shared_ptr<colmap::Point2D>>(m, "Point2D")
        .def(py::init<>())
        .def(py::init([](const Eigen::Vector2d& xy, size_t point3D_id) {
                 std::unique_ptr<Point2D> point2D = std::unique_ptr<Point2D>(new Point2D());
                 point2D->SetXY(xy);
                 point2D->SetPoint3DId(point3D_id);
                 return point2D;
             }),
             py::arg("xy"), py::arg("point3D_id") = kInvalidPoint3DId)
        .def_property("xy", overload_cast_<>()(&Point2D::XY), &Point2D::SetXY)
        .def_property("x", &Point2D::X,
                      [](Point2D& self, double x) { self.SetXY(Eigen::Vector2d(x, self.Y())); })
        .def_property("y", &Point2D::Y,
                      [](Point2D& self, double y) { self.SetXY(Eigen::Vector2d(self.X(), y)); })
        .def_property("point3D_id", &Point2D::Point3DId, &Point2D::SetPoint3DId)
        .def("has_point3D", &Point2D::HasPoint3D)
        .def("__copy__", [](const Point2D& self) { return Point2D(self); })
        .def("__deepcopy__", [](const Point2D& self, py::dict) { return Point2D(self); })
        .def("__repr__", [](const Point2D& self) { return PrintPoint2D(self); });
}