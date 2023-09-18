// Authors: Mihai-Dusmanu (mihaidusmanu), Paul-Edouard Sarlin (skydes)

#include "colmap/estimators/pose.h"
#include "colmap/geometry/rigid3.h"
#include "colmap/math/random.h"
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

py::object absolute_pose_estimation(
    const std::vector<Eigen::Vector2d> points2D,
    const std::vector<Eigen::Vector3d> points3D,
    Camera& camera,
    const AbsolutePoseEstimationOptions estimation_options,
    const AbsolutePoseRefinementOptions refinement_options,
    const bool return_covariance) {
  SetPRNGSeed(0);
  THROW_CHECK_EQ(points2D.size(), points3D.size());
  py::object failure = py::none();
  py::gil_scoped_release release;

  // Absolute pose estimation.
  Rigid3d cam_from_world;
  size_t num_inliers;
  std::vector<char> inlier_mask;

  if (!EstimateAbsolutePose(estimation_options,
                            points2D,
                            points3D,
                            &cam_from_world,
                            &camera,
                            &num_inliers,
                            &inlier_mask)) {
    return failure;
  }

  // Absolute pose refinement.
  Eigen::Matrix<double, 6, 6> covariance;
  if (!RefineAbsolutePose(refinement_options,
                          inlier_mask,
                          points2D,
                          points3D,
                          &cam_from_world,
                          &camera,
                          return_covariance ? &covariance : nullptr)) {
    return failure;
  }

  std::vector<bool> inlier_mask_bool(inlier_mask.begin(), inlier_mask.end());
  py::gil_scoped_acquire acquire;
  py::dict success_dict("cam_from_world"_a = cam_from_world,
                        "num_inliers"_a = num_inliers,
                        "inliers"_a = inlier_mask_bool);
  if (return_covariance) success_dict["covariance"] = covariance;
  return success_dict;
}

py::object pose_refinement(
    const Rigid3d init_cam_from_world,
    const std::vector<Eigen::Vector2d> points2D,
    const std::vector<Eigen::Vector3d> points3D,
    const std::vector<bool> inlier_mask,
    const Camera camera,
    const AbsolutePoseRefinementOptions refinement_options) {
  SetPRNGSeed(0);
  THROW_CHECK_EQ(points2D.size(), points3D.size());
  THROW_CHECK_EQ(inlier_mask.size(), points2D.size());
  py::object failure = py::none();
  py::gil_scoped_release release;

  // Absolute pose estimation.
  Rigid3d refined_cam_from_world = init_cam_from_world;
  std::vector<char> inlier_mask_char;
  for (size_t i = 0; i < inlier_mask.size(); ++i) {
    if (inlier_mask[i]) {
      inlier_mask_char.emplace_back(1);
    } else {
      inlier_mask_char.emplace_back(0);
    }
  }

  // Absolute pose refinement.
  if (!RefineAbsolutePose(refinement_options,
                          inlier_mask_char,
                          points2D,
                          points3D,
                          &refined_cam_from_world,
                          const_cast<Camera*>(&camera))) {
    return failure;
  }

  // Success output dictionary.
  py::gil_scoped_acquire acquire;
  return py::dict("cam_from_world"_a = refined_cam_from_world);
}

void bind_absolute_pose_estimation(py::module& m,
                                   py::class_<RANSACOptions> PyRANSACOptions) {
  auto PyEstimationOptions =
      py::class_<AbsolutePoseEstimationOptions>(m,
                                                "AbsolutePoseEstimationOptions")
          .def(py::init<>([PyRANSACOptions]() {
            AbsolutePoseEstimationOptions options;
            options.estimate_focal_length = false;
            // init through Python to obtain the new defaults defined in
            // __init__
            options.ransac_options = PyRANSACOptions().cast<RANSACOptions>();
            options.ransac_options.max_error = 12.0;
            return options;
          }))
          .def_readwrite("estimate_focal_length",
                         &AbsolutePoseEstimationOptions::estimate_focal_length)
          .def_readwrite(
              "num_focal_length_samples",
              &AbsolutePoseEstimationOptions::num_focal_length_samples)
          .def_readwrite("min_focal_length_ratio",
                         &AbsolutePoseEstimationOptions::min_focal_length_ratio)
          .def_readwrite("max_focal_length_ratio",
                         &AbsolutePoseEstimationOptions::max_focal_length_ratio)
          .def_readwrite("ransac",
                         &AbsolutePoseEstimationOptions::ransac_options);
  make_dataclass(PyEstimationOptions);
  auto est_options =
      PyEstimationOptions().cast<AbsolutePoseEstimationOptions>();

  auto PyRefinementOptions =
      py::class_<AbsolutePoseRefinementOptions>(m,
                                                "AbsolutePoseRefinementOptions")
          .def(py::init<>([]() {
            AbsolutePoseRefinementOptions options;
            options.refine_focal_length = false;
            options.refine_extra_params = false;
            options.print_summary = false;
            return options;
          }))
          .def_readwrite("gradient_tolerance",
                         &AbsolutePoseRefinementOptions::gradient_tolerance)
          .def_readwrite("max_num_iterations",
                         &AbsolutePoseRefinementOptions::max_num_iterations)
          .def_readwrite("loss_function_scale",
                         &AbsolutePoseRefinementOptions::loss_function_scale)
          .def_readwrite("refine_focal_length",
                         &AbsolutePoseRefinementOptions::refine_focal_length)
          .def_readwrite("refine_extra_params",
                         &AbsolutePoseRefinementOptions::refine_extra_params)
          .def_readwrite("print_summary",
                         &AbsolutePoseRefinementOptions::print_summary);
  make_dataclass(PyRefinementOptions);
  auto ref_options =
      PyRefinementOptions().cast<AbsolutePoseRefinementOptions>();

  m.def("absolute_pose_estimation",
        &absolute_pose_estimation,
        "points2D"_a,
        "points3D"_a,
        "camera"_a,
        "estimation_options"_a = est_options,
        "refinement_options"_a = ref_options,
        "return_covariance"_a = false,
        "Absolute pose estimation with non-linear refinement.");

  m.def("pose_refinement",
        &pose_refinement,
        "cam_from_world"_a,
        "points2D"_a,
        "points3D"_a,
        "inlier_mask"_a,
        "camera"_a,
        "refinement_options"_a = ref_options,
        "Non-linear refinement of absolute pose.");
}
