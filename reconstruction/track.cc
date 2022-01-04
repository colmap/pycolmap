// Author: Philipp Lindenberger (Phil26AT)

#include "colmap/base/track.h"
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

void init_track(py::module& m) {
    py::class_<TrackElement, std::shared_ptr<TrackElement>>(m, "TrackElement")
        .def(py::init<>())
        .def(py::init<image_t, point2D_t>())
        .def_readwrite("image_id", &TrackElement::image_id)
        .def_readwrite("point2D_idx", &TrackElement::point2D_idx)
        .def("__copy__", [](const TrackElement& self) { return TrackElement(self); })
        .def("__deepcopy__", [](const TrackElement& self, py::dict) { return TrackElement(self); })
        .def("__repr__", [](const TrackElement& self) {
            return "<TrackElement 'image_id=" + std::to_string(self.image_id) +
                   ",point2D_idx=" + std::to_string(self.point2D_idx) + "'>";
        });

    py::class_<Track, std::shared_ptr<Track>>(m, "Track")
        .def(py::init<>())
        .def(py::init([](const std::vector<TrackElement>& elements) {
            std::unique_ptr<Track> track = std::unique_ptr<Track>(new Track());
            track->AddElements(elements);
            return track;
        }))
        .def("length", &Track::Length, "Track Length.")
        .def("add_element", overload_cast_<image_t, point2D_t>()(&Track::AddElement),
             "Add observation (image_id, point2D_idx) to track.")
        .def("delete_element", overload_cast_<image_t, point2D_t>()(&Track::DeleteElement),
             "Delete observation (image_id, point2D_idx) from track.")
        .def("append", overload_cast_<const TrackElement&>()(&Track::AddElement))
        .def("add_element", overload_cast_<const image_t, const point2D_t>()(&Track::AddElement))
        .def("add_elements", &Track::AddElements, "Add TrackElement list.")
        .def(
            "remove",
            [](Track& self, const size_t idx) {
                THROW_CHECK_LT(idx, self.Elements().size());
                self.DeleteElement(idx);
            },
            "Remove TrackElement at index.")
        .def_property("elements", overload_cast_<>()(&Track::Elements), &Track::SetElements)
        .def("remove", overload_cast_<const image_t, const point2D_t>()(&Track::DeleteElement),
             "Remove TrackElement with (image_id,point2D_idx).")
        .def("__copy__", [](const Track& self) { return Track(self); })
        .def("__deepcopy__", [](const Track& self, py::dict) { return Track(self); })
        .def("__repr__", [](const Track& self) {
            return "<Track 'length=" + std::to_string(self.Length()) + "'>";
        });
}