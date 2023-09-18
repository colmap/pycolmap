// Authors: Mihai-Dusmanu (mihaidusmanu), Paul-Edouard Sarlin (skydes)

#include "colmap/estimators/generalized_pose.h"
#include "colmap/estimators/pose.h"
#include "colmap/geometry/pose.h"
#include "colmap/math/random.h"
#include "colmap/optim/ransac.h"
#include "colmap/scene/camera.h"
#include "colmap/util/misc.h"

#include <fstream>
#include <iostream>

using namespace colmap;

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;

#include "log_exceptions.h"

py::dict rig_absolute_pose_estimation(
    const std::vector<Eigen::Vector2d>& points2D,
    const std::vector<Eigen::Vector3d>& points3D,
    const std::vector<size_t>& camera_idxs,
    const std::vector<Rigid3d>& cams_from_rig,
    std::vector<Camera>& cameras,
    RANSACOptions ransac_options,
    AbsolutePoseRefinementOptions refinement_options,
    const bool return_covariance) {
  SetPRNGSeed(0);
  THROW_CHECK_EQ(points2D.size(), points3D.size());
  THROW_CHECK_EQ(points2D.size(), camera_idxs.size());
  THROW_CHECK_EQ(cams_from_rig.size(), cameras.size());
  THROW_CHECK_GE(*std::min_element(camera_idxs.begin(), camera_idxs.end()), 0);
  THROW_CHECK_LT(*std::max_element(camera_idxs.begin(), camera_idxs.end()),
                 cameras.size());

  py::dict failure_dict("success"_a = false);
  py::gil_scoped_release release;

  Rigid3d rig_from_world;
  size_t num_inliers;
  std::vector<char> inlier_mask;
  if (!EstimateGeneralizedAbsolutePose(ransac_options,
                                       points2D,
                                       points3D,
                                       camera_idxs,
                                       cams_from_rig,
                                       cameras,
                                       &rig_from_world,
                                       &num_inliers,
                                       &inlier_mask)) {
    return failure_dict;
  }

  // Absolute pose refinement.
  Eigen::Matrix<double, 6, 6> covariance;
  if (!RefineGeneralizedAbsolutePose(
          refinement_options,
          inlier_mask,
          points2D,
          points3D,
          camera_idxs,
          cams_from_rig,
          &rig_from_world,
          &cameras,
          return_covariance ? &covariance : nullptr)) {
    return failure_dict;
  }

  // Convert vector<char> to vector<int>.
  std::vector<bool> inlier_mask_bool;
  for (auto it : inlier_mask) {
    if (it) {
      inlier_mask_bool.push_back(true);
    } else {
      inlier_mask_bool.push_back(false);
    }
  }

  py::gil_scoped_acquire acquire;
  py::dict success_dict("success"_a = true,
                        "rig_from_world"_a = rig_from_world,
                        "num_inliers"_a = num_inliers,
                        "inliers"_a = inlier_mask_bool);
  if (return_covariance) success_dict["covariance"] = covariance;
  return success_dict;
}

void bind_generalized_absolute_pose_estimation(py::module& m) {
  auto est_options = m.attr("RANSACOptions")().cast<RANSACOptions>();
  auto ref_options = m.attr("AbsolutePoseRefinementOptions")()
                         .cast<AbsolutePoseRefinementOptions>();

  m.def(
      "rig_absolute_pose_estimation",
      &rig_absolute_pose_estimation,
      "points2D"_a,
      "points3D"_a,
      "cameras"_a,
      "camera_idxs"_a,
      "cams_from_rig"_a,
      "estimation_options"_a = est_options,
      "refinement_options"_a = ref_options,
      "return_covariance"_a = false,
      "Absolute pose estimation with non-linear refinement for a multi-camera "
      "rig.");
}
