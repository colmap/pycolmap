// Authors: Mihai-Dusmanu (mihaidusmanu), Paul-Edouard Sarlin (skydes)

#include <iostream>
#include <fstream>

#include "colmap/base/camera.h"
#include "colmap/base/pose.h"
#include "colmap/base/projection.h"
#include "colmap/base/similarity_transform.h"
#include "colmap/optim/ransac.h"
#include "colmap/util/random.h"
#include "colmap/util/misc.h"
#include "colmap/estimators/pose.h"
#include "colmap/estimators/generalized_absolute_pose.h"

using namespace colmap;

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace py = pybind11;

#include "log_exceptions.h"

const double TOL = 1e-5;

const bool lowerVector3d(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
  if (v1.x() < v2.x()) {
    return true;
  } else if (v1.x() == v2.x()) {
    if (v1.y() < v2.y()) {
      return true;
    } else if (v1.y() == v2.y()) {
      return v1.z() < v2.z();
    } else {
      return false;
    }
  } else {
    return false;
  }
}

const bool equalVector3d(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
  return ((v1 - v2).norm() < TOL);
}

// Measure the support of a model by counting the number of unique inliers (e.g.,
// visible 3D points), number of inliers, and summing all inlier residuals.
// Each sample should have an associated id. Samples with the same id are only
// counted once in num_unique_inliers.
// The support is better if it has more unique inliers, more inliers,
// and a smaller residual sum.
struct UniqueInlierSupportMeasurer {
  struct Support {
    // The number of inliers.
    // This is still needed for determining the dynamic number of iterations.
    size_t num_inliers = 0;

    // The number of unique inliers;
    size_t num_unique_inliers = 0;

    // The sum of all inlier residuals.
    double residual_sum = std::numeric_limits<double>::max();
  };

  void SetSampleIds(const std::vector<size_t>& sample_ids) {
    sample_ids_ = sample_ids;
  }

  // Compute the support of the residuals.
  Support Evaluate(const std::vector<double>& residuals,
                   const double max_residual) {
    CHECK_EQ(residuals.size(), sample_ids_.size());
    Support support;
    support.num_inliers = 0;
    support.num_unique_inliers = 0;
    support.residual_sum = 0;

    std::unordered_set<size_t> inlier_point_ids;

    for (size_t idx = 0; idx < residuals.size(); ++idx) {
      if (residuals[idx] <= max_residual) {
        support.num_inliers += 1;
        inlier_point_ids.insert(sample_ids_[idx]);
        support.residual_sum += residuals[idx];
      }
    }

    support.num_unique_inliers = inlier_point_ids.size();

    return support;
  }

  // Compare the two supports and return the better support.
  bool Compare(const Support& support1, const Support& support2) {
    if (support1.num_unique_inliers > support2.num_unique_inliers) {
      return true;
    } else if (support1.num_unique_inliers == support2.num_unique_inliers) {
      if (support1.num_inliers > support2.num_inliers) {
        return true;
      } else {
        return support1.num_inliers == support2.num_inliers &&
               support1.residual_sum < support2.residual_sum;
      }
    } else {
      return false;
    }
  }

  private:
    std::vector<size_t> sample_ids_;
};

py::dict rig_absolute_pose_estimation(
        const std::vector<std::vector<Eigen::Vector2d>> points2D,
        const std::vector<std::vector<Eigen::Vector3d>> points3D,
        const std::vector<Camera> cameras,
        const std::vector<Eigen::Vector4d> rig_qvecs,
        const std::vector<Eigen::Vector3d> rig_tvecs,
        RANSACOptions ransac_options,
        AbsolutePoseRefinementOptions refinement_options,
        const bool return_covariance
) {
    SetPRNGSeed(0);

    THROW_CHECK_EQ(points2D.size(), points3D.size());
    THROW_CHECK_EQ(points2D.size(), cameras.size());
    THROW_CHECK_EQ(points2D.size(), rig_qvecs.size());
    THROW_CHECK_EQ(points2D.size(), rig_tvecs.size());

    const double max_error_px = ransac_options.max_error;
    THROW_CHECK_GT(max_error_px, 0.);

    // Failure output dictionary.
    py::dict failure_dict;
    failure_dict["success"] = false;
    py::gil_scoped_release release;

    std::vector<GP3PEstimator::X_t> points2D_rig;
    std::vector<Eigen::Vector3d> points3D_all;
    std::vector<Eigen::Vector2d> points2D_all;
    std::vector<size_t> camera_idxs;
    double error_threshold = 0.;
    for (size_t i = 0; i < points2D.size(); ++i) {
        const Eigen::Matrix3x4d rig_tform = ComposeProjectionMatrix(rig_qvecs[i], rig_tvecs[i]);
        for (size_t j = 0; j < points2D[i].size(); ++j) {
            points2D_rig.emplace_back();
            points2D_rig.back().xy = cameras[i].ImageToWorld(points2D[i][j]);
            points2D_rig.back().rel_tform = rig_tform;
            camera_idxs.push_back(i);
        }
        points3D_all.insert(points3D_all.end(), points3D[i].begin(), points3D[i].end());
        points2D_all.insert(points2D_all.end(), points2D[i].begin(), points2D[i].end());

        error_threshold += cameras[i].ImageToWorldThreshold(max_error_px) * points2D[i].size();
    }
    // Average of the errors over the cameras, weighted by the number of correspondences
    error_threshold /= points3D_all.size();
    if (points3D_all.size() == 0) {
        return failure_dict;
    }

    // Associate unique ids to each 3D point.
    // Needed for UniqueInlierSupportMeasurer to avoid counting the same
    // 3D point multiple times due to FoV overlap in rig.
    std::vector<Eigen::Vector3d> unique_points3D = points3D_all;
    std::sort(unique_points3D.begin(), unique_points3D.end(), lowerVector3d);
    unique_points3D.erase(
      std::unique(unique_points3D.begin(), unique_points3D.end(), equalVector3d),
      unique_points3D.end()
    );
    std::vector<size_t> p3D_ids;
    for (auto p3D : points3D_all) {
      p3D_ids.push_back(
        std::lower_bound(unique_points3D.begin(), unique_points3D.end(), p3D, lowerVector3d) - unique_points3D.begin()
      );
    }

    RANSACOptions options(ransac_options);
    options.max_error = error_threshold;
    RANSAC<GP3PEstimator, UniqueInlierSupportMeasurer> ransac(options);
    ransac.support_measurer.SetSampleIds(p3D_ids);
    ransac.estimator.residual_type = GP3PEstimator::ResidualType::ReprojectionError;
    const auto report = ransac.Estimate(points2D_rig, points3D_all);
    if (!report.success) {
        return failure_dict;
    }

    Eigen::Vector3d tvec = report.model.rightCols<1>();
    Eigen::Vector4d qvec = RotationMatrixToQuaternion(report.model.leftCols<3>());

    // Absolute pose refinement.
    Eigen::Matrix<double, 6, 6> covariance;
    if (!RefineGeneralizedAbsolutePose(
          refinement_options, report.inlier_mask,
          points2D_all, points3D_all, camera_idxs,
          rig_qvecs, rig_tvecs, &qvec, &tvec,
          const_cast<std::vector<Camera>*>(&cameras),
          return_covariance ? &covariance : nullptr)) {
        return failure_dict;
    }

    // Convert vector<char> to vector<int>.
    std::vector<bool> inliers;
    for (auto it : report.inlier_mask) {
        if (it) {
            inliers.push_back(true);
        } else {
            inliers.push_back(false);
        }
    }

    py::gil_scoped_acquire acquire;
    py::dict success_dict;
    success_dict["success"] = true;
    success_dict["qvec"] = qvec;
    success_dict["tvec"] = tvec;
    success_dict["num_inliers"] = report.support.num_unique_inliers;
    success_dict["inliers"] = inliers;
    if (return_covariance)
        success_dict["covariance"] = covariance;

    return success_dict;
}

py::dict rig_absolute_pose_estimation(
        const std::vector<std::vector<Eigen::Vector2d>> points2D,
        const std::vector<std::vector<Eigen::Vector3d>> points3D,
        const std::vector<Camera> cameras,
        const std::vector<Eigen::Vector4d> rig_qvecs,
        const std::vector<Eigen::Vector3d> rig_tvecs,
        const double max_error_px,
        const double min_inlier_ratio,
        const int min_num_trials,
        const int max_num_trials,
        const double confidence,
        const bool return_covariance
) {

    RANSACOptions ransac_options;
    ransac_options.max_error = max_error_px;
    ransac_options.min_inlier_ratio = min_inlier_ratio;
    ransac_options.min_num_trials = min_num_trials;
    ransac_options.max_num_trials = max_num_trials;
    ransac_options.confidence = confidence;

    AbsolutePoseRefinementOptions refinement_options;
    refinement_options.refine_focal_length = false;
    refinement_options.refine_extra_params = false;
    refinement_options.print_summary = false;

    return rig_absolute_pose_estimation(
        points2D, points3D, cameras, rig_qvecs, rig_tvecs,
        ransac_options, refinement_options, return_covariance);
}

void bind_generalized_absolute_pose_estimation(py::module& m) {
    auto est_options = m.attr("RANSACOptions")().cast<RANSACOptions>();
    auto ref_options = m.attr("AbsolutePoseRefinementOptions")().cast<AbsolutePoseRefinementOptions>();

    m.def(
        "rig_absolute_pose_estimation",
        static_cast<py::dict (*)(const std::vector<std::vector<Eigen::Vector2d>>,
                                 const std::vector<std::vector<Eigen::Vector3d>>,
                                 const std::vector<Camera>,
                                 const std::vector<Eigen::Vector4d>,
                                 const std::vector<Eigen::Vector3d>,
                                 const RANSACOptions,
                                 const AbsolutePoseRefinementOptions,
                                 bool
                                 )>(&rig_absolute_pose_estimation),
        py::arg("points2D"), py::arg("points3D"),
        py::arg("cameras"), py::arg("rig_qvecs"), py::arg("rig_tvecs"),
        py::arg("estimation_options") = est_options,
        py::arg("refinement_options") = ref_options,
        py::arg("return_covariance") = false,
        "Absolute pose estimation with non-linear refinement for a multi-camera rig.");

    m.def(
        "rig_absolute_pose_estimation",
        static_cast<py::dict (*)(const std::vector<std::vector<Eigen::Vector2d>>,
                                 const std::vector<std::vector<Eigen::Vector3d>>,
                                 const std::vector<Camera>,
                                 const std::vector<Eigen::Vector4d>,
                                 const std::vector<Eigen::Vector3d>,
                                 const double, const double,
                                 const int, const int, const double, const bool
                                 )>(&rig_absolute_pose_estimation),
        py::arg("points2D"), py::arg("points3D"),
        py::arg("cameras"), py::arg("rig_qvecs"), py::arg("rig_tvecs"),
        py::arg("max_error_px") = est_options.max_error,
        py::arg("min_inlier_ratio") = est_options.min_inlier_ratio,
        py::arg("min_num_trials") = est_options.min_num_trials,
        py::arg("max_num_trials") = est_options.max_num_trials,
        py::arg("confidence") = est_options.confidence,
        py::arg("return_covariance") = false,
        "Absolute pose estimation with non-linear refinement for a multi-camera rig.");
}
