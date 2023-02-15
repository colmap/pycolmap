// Author: Mihai Dusmanu (mihaidusmanu)

#include "colmap/sfm/incremental_triangulator.h"

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

void init_incremental_triangulator(py::module& m) {
    py::class_<IncrementalTriangulator::Options, std::shared_ptr<IncrementalTriangulator::Options>>(m, "IncrementalTriangulatorOptions")
        .def(py::init<>())
        .def_readwrite("max_transitivity", &IncrementalTriangulator::Options::max_transitivity)
        .def_readwrite("create_max_angle_error", &IncrementalTriangulator::Options::create_max_angle_error)
        .def_readwrite("continue_max_angle_error", &IncrementalTriangulator::Options::continue_max_angle_error)
        .def_readwrite("merge_max_reproj_error", &IncrementalTriangulator::Options::merge_max_reproj_error)
        .def_readwrite("complete_max_reproj_error", &IncrementalTriangulator::Options::complete_max_reproj_error)
        .def_readwrite("complete_max_transitivity", &IncrementalTriangulator::Options::complete_max_transitivity)
        .def_readwrite("re_max_angle_error", &IncrementalTriangulator::Options::re_max_angle_error)
        .def_readwrite("re_min_ratio", &IncrementalTriangulator::Options::re_min_ratio)
        .def_readwrite("re_max_trials", &IncrementalTriangulator::Options::re_max_trials)
        .def_readwrite("min_angle", &IncrementalTriangulator::Options::min_angle)
        .def_readwrite("ignore_two_view_track", &IncrementalTriangulator::Options::ignore_two_view_tracks)
        .def_readwrite("min_focal_length_ratio", &IncrementalTriangulator::Options::min_focal_length_ratio)
        .def_readwrite("max_focal_length_ratio", &IncrementalTriangulator::Options::max_focal_length_ratio)
        .def_readwrite("max_extra_param", &IncrementalTriangulator::Options::max_extra_param);

    py::class_<IncrementalTriangulator, std::shared_ptr<IncrementalTriangulator>>(m, "IncrementalTriangulator")
        .def(py::init<CorrespondenceGraph*, Reconstruction*>())
        .def("triangulate_image", &IncrementalTriangulator::TriangulateImage)
        .def("complete_image", &IncrementalTriangulator::CompleteImage)
        .def("complete_all_tracks", &IncrementalTriangulator::CompleteAllTracks)
        .def("merge_all_tracks", &IncrementalTriangulator::MergeAllTracks)
        .def("retriangulate", &IncrementalTriangulator::Retriangulate)
        .def("add_modified_point3D", &IncrementalTriangulator::AddModifiedPoint3D)
        .def("clear_modified_points3D", &IncrementalTriangulator::ClearModifiedPoints3D)
        .def("merge_tracks", &IncrementalTriangulator::MergeTracks)
        .def("complete_tracks", &IncrementalTriangulator::CompleteTracks)
        // Missing bindings: GetModifiedPoints3D
        // Private bindings: Find, Create, Continue, Merge, Complete, HasCameraBogusParams
        .def("__copy__", [](const IncrementalTriangulator& self) { return IncrementalTriangulator(self); })
        .def("__deepcopy__", [](const IncrementalTriangulator& self, py::dict) { return IncrementalTriangulator(self); })
        .def("__repr__", [](const IncrementalTriangulator& self) {
                 std::stringstream ss;
                 ss << "<IncrementalTriangulator>";
                 return ss.str();
        });
}