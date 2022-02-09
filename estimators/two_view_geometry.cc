// Authors: John Lambert (johnwlambert), Paul-Edouard Sarlin (skydes)

#include <iostream>
#include <fstream>

#include "colmap/base/camera.h"
#include "colmap/base/pose.h"
#include "colmap/estimators/two_view_geometry.h"
#include "colmap/optim/loransac.h"
#include "colmap/util/random.h"

using namespace colmap;

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace py = pybind11;

#include "log_exceptions.h"
#include "helpers.h"


py::dict two_view_geometry_estimation(
        const std::vector<Eigen::Vector2d> points2D1,
        const std::vector<Eigen::Vector2d> points2D2,
        Camera& camera1,
        Camera& camera2,
        TwoViewGeometry::Options options
) {
    SetPRNGSeed(0);

    // Check that both vectors have the same size.
    THROW_CHECK_EQ(points2D1.size(), points2D2.size());

    // Failure output dictionary.
    py::dict failure_dict;
    failure_dict["success"] = false;
    py::gil_scoped_release release;

    FeatureMatches matches;
    matches.reserve(points2D1.size());

    for (size_t i=0; i < points2D1.size(); i++) {
       matches.emplace_back(i,i);
    }

    TwoViewGeometry two_view_geometry;

    two_view_geometry.EstimateCalibrated(
            camera1, points2D1, camera2, points2D2, matches, options);

    if (!two_view_geometry.EstimateRelativePose(camera1, points2D1, camera2, points2D2)) {
        return failure_dict;
    }
    const FeatureMatches inlier_matches = two_view_geometry.inlier_matches;

    // Convert vector<char> to vector<int>.
    std::vector<bool> inliers(points2D1.size(), false);
    for (auto m : inlier_matches) {
        inliers[m.point2D_idx1] = true;
    }

    // Success output dictionary.
    // See https://github.com/colmap/colmap/blob/dev/src/estimators/two_view_geometry.h#L48
    // for a definition of the different configuration types.
    py::gil_scoped_acquire acquire;
    py::dict success_dict;
    success_dict["success"] = true;
    success_dict["configuration_type"] = two_view_geometry.config;
    success_dict["qvec"] = two_view_geometry.qvec;
    success_dict["tvec"] = two_view_geometry.tvec;
    success_dict["num_inliers"] = inlier_matches.size();
    success_dict["inliers"] = inliers;

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
    TwoViewGeometry::Options two_view_geometry_options;
    two_view_geometry_options.ransac_options.max_error = max_error_px;
    two_view_geometry_options.ransac_options.min_inlier_ratio = min_inlier_ratio;
    two_view_geometry_options.ransac_options.min_num_trials = min_num_trials;
    two_view_geometry_options.ransac_options.max_num_trials = max_num_trials;
    two_view_geometry_options.ransac_options.confidence = confidence;
    return two_view_geometry_estimation(
            points2D1, points2D2, camera1, camera2, two_view_geometry_options);
}

void bind_two_view_geometry_estimation(py::module& m, py::class_<RANSACOptions> PyRANSACOptions) {
    auto PyEstimationOptions =
        py::class_<TwoViewGeometry::Options>(m, "TwoViewGeometryOptions")
        .def(py::init<>([PyRANSACOptions]() {
            TwoViewGeometry::Options options;
            // init through Python to obtain the new defaults defined in __init__
            options.ransac_options = PyRANSACOptions().cast<RANSACOptions>();
            return options;
        }))
        .def_readwrite("min_num_inliers", &TwoViewGeometry::Options::min_num_inliers)
        .def_readwrite("min_E_F_inlier_ratio", &TwoViewGeometry::Options::min_E_F_inlier_ratio)
        .def_readwrite("max_H_inlier_ratio", &TwoViewGeometry::Options::max_H_inlier_ratio)
        .def_readwrite("watermark_min_inlier_ratio", &TwoViewGeometry::Options::watermark_min_inlier_ratio)
        .def_readwrite("watermark_border_size", &TwoViewGeometry::Options::watermark_border_size)
        .def_readwrite("detect_watermark", &TwoViewGeometry::Options::detect_watermark)
        .def_readwrite("multiple_ignore_watermark", &TwoViewGeometry::Options::multiple_ignore_watermark)
        .def_readwrite("ransac", &TwoViewGeometry::Options::ransac_options);
    make_dataclass(PyEstimationOptions);
    auto est_options = PyEstimationOptions().cast<TwoViewGeometry::Options>();

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
                                 const TwoViewGeometry::Options
                                 )>(&two_view_geometry_estimation),
        py::arg("points2D1"), py::arg("points2D2"),
        py::arg("camera1"), py::arg("camera2"),
        py::arg("estimation_options") = est_options,
        "Generic two-view geometry estimation");

    m.def(
        "two_view_geometry_estimation",
        static_cast<py::dict (*)(const std::vector<Eigen::Vector2d>,
                                 const std::vector<Eigen::Vector2d>,
                                 Camera&, Camera&,
                                 const double, const double,
                                 const int, const int, const double
                                 )>(&two_view_geometry_estimation),
        py::arg("points2D1"), py::arg("points2D2"),
        py::arg("camera1"), py::arg("camera2"),
        py::arg("max_error_px") = est_options.ransac_options.max_error,
        py::arg("min_inlier_ratio") = est_options.ransac_options.min_inlier_ratio,
        py::arg("min_num_trials") = est_options.ransac_options.min_num_trials,
        py::arg("max_num_trials") = est_options.ransac_options.max_num_trials,
        py::arg("confidence") = est_options.ransac_options.confidence,
        "Generic two-view geometry estimation");
}
