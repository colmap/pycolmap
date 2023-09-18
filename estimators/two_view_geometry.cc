// Authors: John Lambert (johnwlambert), Paul-Edouard Sarlin (skydes)

#include "colmap/scene/two_view_geometry.h"

#include "colmap/estimators/two_view_geometry.h"
#include "colmap/geometry/pose.h"
#include "colmap/math/random.h"
#include "colmap/optim/loransac.h"
#include "colmap/scene/camera.h"

#include <fstream>
#include <iostream>

using namespace colmap;

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;

#include "helpers.h"
#include "log_exceptions.h"
#include "utils.h"

py::object two_view_geometry_estimation(
    const Camera& camera1,
    const std::vector<Eigen::Vector2d>& points1,
    const Camera& camera2,
    const std::vector<Eigen::Vector2d>& points2,
    TwoViewGeometryOptions options) {
  SetPRNGSeed(0);
  THROW_CHECK_EQ(points1.size(), points2.size());
  py::object failure = py::none();
  py::gil_scoped_release release;

  FeatureMatches matches;
  matches.reserve(points1.size());
  for (size_t i = 0; i < points1.size(); i++) {
    matches.emplace_back(i, i);
  }

  auto two_view_geometry = EstimateCalibratedTwoViewGeometry(
      camera1, points1, camera2, points2, matches, options);

  if (!EstimateTwoViewGeometryPose(
          camera1, points1, camera2, points2, &two_view_geometry)) {
    return failure;
  }
  const FeatureMatches& inlier_matches = two_view_geometry.inlier_matches;
  Eigen::VectorX<bool> inlier_mask(points1.size());
  inlier_mask.setConstant(false);
  for (auto m : inlier_matches) {
    inlier_mask(m.point2D_idx1) = true;
  }
  py::gil_scoped_acquire acquire;
  return py::dict("configuration_type"_a = two_view_geometry.config,
                  "cam2_from_cam1"_a = two_view_geometry.cam2_from_cam1,
                  "num_inliers"_a = inlier_matches.size(),
                  "inliers"_a = inlier_mask);
}

void bind_two_view_geometry_estimation(py::module& m) {
  auto PyTwoViewGeometryOptions =
      py::class_<TwoViewGeometryOptions>(m, "TwoViewGeometryOptions")
          .def(py::init<>())
          .def_readwrite("min_num_inliers",
                         &TwoViewGeometryOptions::min_num_inliers)
          .def_readwrite("min_E_F_inlier_ratio",
                         &TwoViewGeometryOptions::min_E_F_inlier_ratio)
          .def_readwrite("max_H_inlier_ratio",
                         &TwoViewGeometryOptions::max_H_inlier_ratio)
          .def_readwrite("watermark_min_inlier_ratio",
                         &TwoViewGeometryOptions::watermark_min_inlier_ratio)
          .def_readwrite("watermark_border_size",
                         &TwoViewGeometryOptions::watermark_border_size)
          .def_readwrite("detect_watermark",
                         &TwoViewGeometryOptions::detect_watermark)
          .def_readwrite("multiple_ignore_watermark",
                         &TwoViewGeometryOptions::multiple_ignore_watermark)
          .def_readwrite("force_H_use", &TwoViewGeometryOptions::force_H_use)
          .def_readwrite("compute_relative_pose",
                         &TwoViewGeometryOptions::compute_relative_pose)
          .def_readwrite("multiple_models",
                         &TwoViewGeometryOptions::multiple_models)
          .def_readwrite("ransac", &TwoViewGeometryOptions::ransac_options);
  make_dataclass(PyTwoViewGeometryOptions);
  auto tvg_options = PyTwoViewGeometryOptions().cast<TwoViewGeometryOptions>();

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

  m.def("two_view_geometry_estimation",
        &two_view_geometry_estimation,
        "camera1"_a,
        "points1"_a,
        "camera2"_a,
        "points2"_a,
        "estimation_options"_a = tvg_options,
        "Generic two-view geometry estimation");
}
