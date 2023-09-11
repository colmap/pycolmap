// Author: Philipp Lindenberger (Phil26AT)

// Use Eigens aligned allocator for vectors
#include <Eigen/StdVector>

#include "colmap/scene/point2d.h"
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

using vector_Point2D = std::vector<class Point2D, Eigen::aligned_allocator<Point2D>>;
PYBIND11_MAKE_OPAQUE(vector_Point2D);

std::string PrintPoint2D(const Point2D& p2D) {
    std::stringstream ss;
    ss << "<Point2D 'xy=[" << p2D.xy.transpose()
       << "], point3D_id=" << (p2D.HasPoint3D() ? std::to_string(p2D.point3D_id) : "Invalid")
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

    py::class_<Point2D, std::shared_ptr<Point2D>>(m, "Point2D")
        .def(py::init<>())
        .def(py::init<const Eigen::Vector2d&, size_t>(),
             py::arg("xy"), py::arg("point3D_id") = kInvalidPoint3DId)
        .def_readwrite("xy", &Point2D::xy)
        .def_readwrite("point3D_id", &Point2D::point3D_id)
        .def("has_point3D", &Point2D::HasPoint3D)
        .def("__copy__", [](const Point2D& self) { return Point2D(self); })
        .def("__deepcopy__", [](const Point2D& self, py::dict) { return Point2D(self); })
        .def("__repr__", &PrintPoint2D);
}
