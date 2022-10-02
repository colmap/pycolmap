#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace py = pybind11;

#include <colmap/base/pose.h>

#include "estimators/absolute_pose.cc"
#include "estimators/generalized_absolute_pose.cc"
#include "estimators/essential_matrix.cc"
#include "estimators/fundamental_matrix.cc"
#include "estimators/two_view_geometry.cc"
#include "estimators/homography.cc"

#include "homography_decomposition.cc"
#include "transformations.cc"
#include "sift.cc"
#include "helpers.h"
#include "utils.h"

#include "pipeline/sfm.cc"
#include "pipeline/mvs.cc"
#include "pipeline/meshing.cc"

#include "reconstruction/reconstruction.cc"
#include "reconstruction/correspondence_graph.cc"
#include "reconstruction/incremental_triangulator.cc"

void init_reconstruction(py::module &);
void init_transforms(py::module &);

PYBIND11_MODULE(pycolmap, m) {
    m.doc() = "COLMAP plugin";
#ifdef VERSION_INFO
    m.attr("__version__") = py::str(VERSION_INFO);
#else
    m.attr("__version__") = py::str("dev");
#endif

    auto PyDevice = py::enum_<Device>(m, "Device")
        .value("auto", Device::AUTO)
        .value("cpu", Device::CPU)
        .value("cuda", Device::CUDA);
    AddStringToEnumConstructor(PyDevice);

    m.attr("has_cuda") = IsGPU(Device::AUTO);

    // Estimators
    auto PyRANSACOptions = py::class_<RANSACOptions>(m, "RANSACOptions")
        .def(py::init<>([]() {
            RANSACOptions options;
            options.max_error = 4.0;
            options.min_inlier_ratio = 0.01;
            options.confidence = 0.9999;
            options.min_num_trials = 1000;
            options.max_num_trials = 100000;
            return options;
        }))
        .def_readwrite("max_error", &RANSACOptions::max_error)
        .def_readwrite("min_inlier_ratio", &RANSACOptions::min_inlier_ratio)
        .def_readwrite("confidence", &RANSACOptions::confidence)
        .def_readwrite("dyn_num_trials_multiplier", &RANSACOptions::dyn_num_trials_multiplier)
        .def_readwrite("min_num_trials", &RANSACOptions::min_num_trials)
        .def_readwrite("max_num_trials", &RANSACOptions::max_num_trials);
    make_dataclass(PyRANSACOptions);

    bind_absolute_pose_estimation(m, PyRANSACOptions);
    bind_essential_matrix_estimation(m);
    bind_fundamental_matrix_estimation(m);
    bind_generalized_absolute_pose_estimation(m);
    bind_homography_estimation(m);
    bind_two_view_geometry_estimation(m, PyRANSACOptions);

    // Homography Decomposition.
    m.def("homography_decomposition", &homography_decomposition_estimation,
          py::arg("H"),
          py::arg("K1"),
          py::arg("K2"),
          py::arg("points1"),
          py::arg("points2"),
          "Analytical Homography Decomposition.");

    // Reconstruction bindings
    init_reconstruction(m);

    // Correspondence graph bindings
    init_correspondence_graph(m);

    // Incremental triangulator bindings
    init_incremental_triangulator(m);

    // Automatic conversion from python dicts to colmap cameras for backwards compatibility
    py::implicitly_convertible<py::dict, colmap::Camera>();

    // Transformation Bindings
    init_transforms(m);

    // Main reconstruction steps
    init_sfm(m);
    init_mvs(m);
    init_meshing(m);

    // SIFT feature detector and descriptor
    init_sift(m);

    py::add_ostream_redirect(m, "ostream");
}
