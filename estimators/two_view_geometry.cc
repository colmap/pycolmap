// Authors: John Lambert (johnwlambert), Paul-Edouard Sarlin (skydes)

#include <iostream>
#include <fstream>

#include "colmap/scene/camera.h"
#include "colmap/scene/two_view_geometry.h"
#include "colmap/geometry/pose.h"
#include "colmap/estimators/two_view_geometry.h"
#include "colmap/optim/loransac.h"
#include "colmap/math/random.h"

using namespace colmap;

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace py = pybind11;
using namespace pybind11::literals;

#include "log_exceptions.h"
#include "helpers.h"


py::dict two_view_geometry_estimation(
        const std::vector<Eigen::Vector2d> points2D1,
        const std::vector<Eigen::Vector2d> points2D2,
        Camera& camera1,
        Camera& camera2,
        TwoViewGeometryOptions options
) {
    SetPRNGSeed(0);

    // Check that both vectors have the same size.
    THROW_CHECK_EQ(points2D1.size(), points2D2.size());

    // Failure output dictionary.
    py::dict failure_dict("success"_a = false);
    py::gil_scoped_release release;

    FeatureMatches matches;
    matches.reserve(points2D1.size());

    for (size_t i=0; i < points2D1.size(); i++) {
       matches.emplace_back(i,i);
    }

    auto two_view_geometry = EstimateCalibratedTwoViewGeometry(
            camera1, points2D1, camera2, points2D2, matches, options);

    if (!EstimateTwoViewGeometryPose(camera1, points2D1, camera2, points2D2, &two_view_geometry)) {
        return failure_dict;
    }
    const FeatureMatches inlier_matches = two_view_geometry.inlier_matches;

    // Convert vector<char> to vector<int>.
    std::vector<bool> inliers(points2D1.size(), false);
    for (auto m : inlier_matches) {
        inliers[m.point2D_idx1] = true;
    }

    // Success output dictionary.
    py::gil_scoped_acquire acquire;
    py::dict success_dict("success"_a = true,
                          "configuration_type"_a = two_view_geometry.config,
                          "cam2_from_cam1"_a = two_view_geometry.cam2_from_cam1,
                          "num_inliers"_a = inlier_matches.size(),
                          "inliers"_a = inliers);
    return success_dict;
}

py::dict two_view_geometry_estimation(
        const std::vector<Eigen::Vector2d> points2D1,
        const std::vector<Eigen::Vector2d> points2D2,
        Camera& camera1,
        Camera& camera2,
        const double max_error_px,
        const double min_inlier_ratio,
        const int min_num_trials,
        const int max_num_trials,
        const double confidence
) {
    TwoViewGeometryOptions two_view_geometry_options;
    two_view_geometry_options.ransac_options.max_error = max_error_px;
    two_view_geometry_options.ransac_options.min_inlier_ratio = min_inlier_ratio;
    two_view_geometry_options.ransac_options.min_num_trials = min_num_trials;
    two_view_geometry_options.ransac_options.max_num_trials = max_num_trials;
    two_view_geometry_options.ransac_options.confidence = confidence;
    return two_view_geometry_estimation(
            points2D1, points2D2, camera1, camera2, two_view_geometry_options);
}

void bind_two_view_geometry_estimation(py::module& m) {
    auto PyEstimationOptions =
        py::class_<TwoViewGeometryOptions>(m, "TwoViewGeometryOptions")
        .def(py::init<>())
        .def_readwrite("min_num_inliers", &TwoViewGeometryOptions::min_num_inliers)
        .def_readwrite("min_E_F_inlier_ratio", &TwoViewGeometryOptions::min_E_F_inlier_ratio)
        .def_readwrite("max_H_inlier_ratio", &TwoViewGeometryOptions::max_H_inlier_ratio)
        .def_readwrite("watermark_min_inlier_ratio", &TwoViewGeometryOptions::watermark_min_inlier_ratio)
        .def_readwrite("watermark_border_size", &TwoViewGeometryOptions::watermark_border_size)
        .def_readwrite("detect_watermark", &TwoViewGeometryOptions::detect_watermark)
        .def_readwrite("multiple_ignore_watermark", &TwoViewGeometryOptions::multiple_ignore_watermark)
        .def_readwrite("force_H_use", &TwoViewGeometryOptions::force_H_use)
        .def_readwrite("compute_relative_pose", &TwoViewGeometryOptions::compute_relative_pose)
        .def_readwrite("multiple_models", &TwoViewGeometryOptions::multiple_models)
        .def_readwrite("ransac", &TwoViewGeometryOptions::ransac_options);
    make_dataclass(PyEstimationOptions);
    auto est_options = PyEstimationOptions().cast<TwoViewGeometryOptions>();

    py::enum_<TwoViewGeometry::ConfigurationType>(m, "TwoViewGeometry")
        .value("UNDEFINED", TwoViewGeometry::UNDEFINED)
        .value("DEGENERATE", TwoViewGeometry::DEGENERATE)
        .value("CALIBRATED", TwoViewGeometry::CALIBRATED)
        .value("UNCALIBRATED", TwoViewGeometry::UNCALIBRATED)
        .value("PLANAR", TwoViewGeometry::PLANAR)
        .value("PANORAMIC", TwoViewGeometry::PANORAMIC)
        .value("PLANAR_OR_PANORAMIC", TwoViewGeometry::PLANAR_OR_PANORAMIC)
        .value("WATERMARK", TwoViewGeometry::WATERMARK)
        .value("MULTIPLE", TwoViewGeometry::MULTIPLE);

    m.def(
        "two_view_geometry_estimation",
        static_cast<py::dict (*)(const std::vector<Eigen::Vector2d>,
                                 const std::vector<Eigen::Vector2d>,
                                 Camera&, Camera&,
                                 const TwoViewGeometryOptions
                                 )>(&two_view_geometry_estimation),
        "points2D1"_a, "points2D2"_a,
        "camera1"_a, "camera2"_a,
        "estimation_options"_a = est_options,
        "Generic two-view geometry estimation");

    m.def(
        "two_view_geometry_estimation",
        static_cast<py::dict (*)(const std::vector<Eigen::Vector2d>,
                                 const std::vector<Eigen::Vector2d>,
                                 Camera&, Camera&,
                                 const double, const double,
                                 const int, const int, const double
                                 )>(&two_view_geometry_estimation),
        "points2D1"_a, "points2D2"_a,
        "camera1"_a, "camera2"_a,
        "max_error_px"_a = est_options.ransac_options.max_error,
        "min_inlier_ratio"_a = est_options.ransac_options.min_inlier_ratio,
        "min_num_trials"_a = est_options.ransac_options.min_num_trials,
        "max_num_trials"_a = est_options.ransac_options.max_num_trials,
        "confidence"_a = est_options.ransac_options.confidence,
        "Generic two-view geometry estimation");
}
