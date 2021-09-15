#include <iostream>
#include <fstream>

#include "colmap/base/camera.h"
#include "colmap/base/pose.h"
#include "colmap/base/projection.h"
#include "colmap/base/similarity_transform.h"
#include "colmap/optim/ransac.h"
#include "colmap/util/random.h"
#include "colmap/util/misc.h"
#include "colmap/estimators/generalized_absolute_pose.h"

#include "colmap/base/camera_models.h"
#include "colmap/optim/bundle_adjustment.h"
#include "colmap/base/cost_functions.h"

using namespace colmap;

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace py = pybind11;

#include "log_exceptions.h"

struct GeneralizedAbsolutePoseRefinementOptions {
  // Convergence criterion.
  double gradient_tolerance = 1.0;

  // Maximum number of solver iterations.
  int max_num_iterations = 100;

  // Scaling factor determines at which residual robustification takes place.
  double loss_function_scale = 1.0;

  // Whether to refine the focal length parameter group.
  bool refine_focal_length = true;

  // Whether to refine the extra parameter group.
  bool refine_extra_params = true;

  // Whether to print final summary.
  bool print_summary = true;

  void Check() const {
    THROW_CHECK_GE(gradient_tolerance, 0.0);
    THROW_CHECK_GE(max_num_iterations, 0);
    THROW_CHECK_GE(loss_function_scale, 0.0);
  }
};

bool RefineGeneralizedAbsolutePose(
                        const GeneralizedAbsolutePoseRefinementOptions& options,
                        const std::vector<char>& inlier_mask,
                        const std::vector<Eigen::Vector2d>& points2D,
                        const std::vector<Eigen::Vector3d>& points3D,
                        const std::vector<size_t>& camera_idxs,
                        const std::vector<Eigen::Vector4d>& rig_qvecs,
                        const std::vector<Eigen::Vector3d>& rig_tvecs,
                        Eigen::Vector4d* qvec, Eigen::Vector3d* tvec,
                        std::vector<Camera>* cameras) {
  THROW_CHECK_EQ(points2D.size(), inlier_mask.size());
  THROW_CHECK_EQ(points2D.size(), points3D.size());
  THROW_CHECK_EQ(points2D.size(), camera_idxs.size());
  THROW_CHECK_EQ(rig_qvecs.size(), rig_tvecs.size());
  THROW_CHECK_EQ(rig_qvecs.size(), cameras->size());
  THROW_CHECK_GE(*std::min_element(camera_idxs.begin(), camera_idxs.end()), 0);
  THROW_CHECK_LT(*std::max_element(camera_idxs.begin(), camera_idxs.end()), cameras->size());
  options.Check();

  ceres::LossFunction* loss_function =
      new ceres::CauchyLoss(options.loss_function_scale);

  std::vector<double*> cameras_params_data;
  for (size_t i = 0; i < cameras->size(); i++) {
    cameras_params_data.push_back(cameras->at(i).ParamsData());
  }
  std::vector<size_t> camera_counts(cameras->size(), 0);
  double* qvec_data = qvec->data();
  double* tvec_data = tvec->data();

  std::vector<Eigen::Vector3d> points3D_copy = points3D;
  std::vector<Eigen::Vector4d> rig_qvecs_copy = rig_qvecs;
  std::vector<Eigen::Vector3d> rig_tvecs_copy = rig_tvecs;

  ceres::Problem problem;

  for (size_t i = 0; i < points2D.size(); ++i) {
    // Skip outlier observations
    if (!inlier_mask[i]) {
      continue;
    }
    size_t camera_idx = camera_idxs[i];
    camera_counts[camera_idx] += 1;

    ceres::CostFunction* cost_function = nullptr;
    switch (cameras->at(camera_idx).ModelId()) {
#define CAMERA_MODEL_CASE(CameraModel)                                  \
  case CameraModel::kModelId:                                           \
    cost_function =                                                     \
        RigBundleAdjustmentCostFunction<CameraModel>::Create(points2D[i]); \
    break;

      CAMERA_MODEL_SWITCH_CASES

#undef CAMERA_MODEL_CASE
    }

    problem.AddResidualBlock(cost_function, loss_function,
                             qvec_data, tvec_data,
                             rig_qvecs_copy[camera_idx].data(),
                             rig_tvecs_copy[camera_idx].data(),
                             points3D_copy[i].data(),
                             cameras_params_data[camera_idx]);
    problem.SetParameterBlockConstant(points3D_copy[i].data());
  }

  if (problem.NumResiduals() > 0) {
    // Quaternion parameterization.
    *qvec = NormalizeQuaternion(*qvec);
    ceres::LocalParameterization* quaternion_parameterization =
        new ceres::QuaternionParameterization;
    problem.SetParameterization(qvec_data, quaternion_parameterization);

    // Camera parameterization.
    for (size_t i = 0; i < cameras->size(); i++) {
      if (camera_counts[i] == 0)
        continue;
      Camera& camera = cameras->at(i);

      // We don't optimize the rig parameters (it's likely very unconstrainted)
      problem.SetParameterBlockConstant(rig_qvecs_copy[i].data());
      problem.SetParameterBlockConstant(rig_tvecs_copy[i].data());

      if (!options.refine_focal_length && !options.refine_extra_params) {
        problem.SetParameterBlockConstant(camera.ParamsData());
      } else {
        // Always set the principal point as fixed.
        std::vector<int> camera_params_const;
        const std::vector<size_t>& principal_point_idxs =
            camera.PrincipalPointIdxs();
        camera_params_const.insert(camera_params_const.end(),
                                   principal_point_idxs.begin(),
                                   principal_point_idxs.end());

        if (!options.refine_focal_length) {
          const std::vector<size_t>& focal_length_idxs =
              camera.FocalLengthIdxs();
          camera_params_const.insert(camera_params_const.end(),
                                     focal_length_idxs.begin(),
                                     focal_length_idxs.end());
        }

        if (!options.refine_extra_params) {
          const std::vector<size_t>& extra_params_idxs =
              camera.ExtraParamsIdxs();
          camera_params_const.insert(camera_params_const.end(),
                                     extra_params_idxs.begin(),
                                     extra_params_idxs.end());
        }

        if (camera_params_const.size() == camera.NumParams()) {
          problem.SetParameterBlockConstant(camera.ParamsData());
        } else {
          ceres::SubsetParameterization* camera_params_parameterization =
              new ceres::SubsetParameterization(
                  static_cast<int>(camera.NumParams()), camera_params_const);
          problem.SetParameterization(camera.ParamsData(),
                                      camera_params_parameterization);
        }
      }
    }
  }

  ceres::Solver::Options solver_options;
  solver_options.gradient_tolerance = options.gradient_tolerance;
  solver_options.max_num_iterations = options.max_num_iterations;
  solver_options.linear_solver_type = ceres::DENSE_QR;
  //solver_options.minimizer_progress_to_stdout = true;  // fixme

  // The overhead of creating threads is too large.
  solver_options.num_threads = 1;
#if CERES_VERSION_MAJOR < 2
  solver_options.num_linear_solver_threads = 1;
#endif  // CERES_VERSION_MAJOR

  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);

  if (solver_options.minimizer_progress_to_stdout) {
    std::cout << std::endl;
  }

  if (options.print_summary) {
    PrintHeading2("Pose refinement report");
    PrintSolverSummary(summary);
  }

  return summary.IsSolutionUsable();
}

py::dict rig_absolute_pose_estimation(
        const std::vector<std::vector<Eigen::Vector2d>> points2D,
        const std::vector<std::vector<Eigen::Vector3d>> points3D,
        const std::vector<Camera> cameras,
        const std::vector<Eigen::Vector4d> rig_qvecs,
        const std::vector<Eigen::Vector3d> rig_tvecs,
        const double max_error_px
) {
    SetPRNGSeed(0);

    THROW_CHECK_EQ(points2D.size(), points3D.size());
    THROW_CHECK_EQ(points2D.size(), cameras.size());
    THROW_CHECK_EQ(points2D.size(), rig_qvecs.size());
    THROW_CHECK_EQ(points2D.size(), rig_tvecs.size());
    THROW_CHECK_GT(max_error_px, 0.);

    // Failure output dictionary.
    py::dict failure_dict;
    failure_dict["success"] = false;
    if (points2D.size() == 0) {
        return failure_dict;
    }

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
    // average of the errors over the cameras, weighted by the number of correspondences
    error_threshold /= points3D_all.size();

    RANSACOptions options;
    options.max_error = error_threshold;
    options.min_inlier_ratio = 0.01;
    options.min_num_trials = 1000;
    options.max_num_trials = 100000;
    options.confidence = 0.9999;
    RANSAC<GP3PEstimator> ransac(options);
    ransac.estimator.residual_type = GP3PEstimator::ResidualType::ReprojectionError;
    const auto report = ransac.Estimate(points2D_rig, points3D_all);
    size_t num_inliers = report.support.num_inliers;
    if (num_inliers == 0) {
        return failure_dict;
    }

    Eigen::Vector3d tvec = report.model.rightCols<1>();
    Eigen::Vector4d qvec = RotationMatrixToQuaternion(report.model.leftCols<3>());

    GeneralizedAbsolutePoseRefinementOptions abs_pose_refinement_options;
    abs_pose_refinement_options.refine_focal_length = false;
    abs_pose_refinement_options.refine_extra_params = false;
    abs_pose_refinement_options.print_summary = false;

    // Absolute pose refinement.
    if (!RefineGeneralizedAbsolutePose(
          abs_pose_refinement_options, report.inlier_mask,
          points2D_all, points3D_all, camera_idxs,
          rig_qvecs, rig_tvecs, &qvec, &tvec, 
          const_cast<std::vector<Camera>*>(&cameras))) {
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

    py::dict success_dict;
    success_dict["success"] = true;
    success_dict["qvec"] = qvec;
    success_dict["tvec"] = tvec;
    success_dict["num_inliers"] = num_inliers;
    success_dict["inliers"] = inliers;

    return success_dict;
}

py::dict rig_absolute_pose_estimation_camera_dicts(
        const std::vector<std::vector<Eigen::Vector2d>> points2D,
        const std::vector<std::vector<Eigen::Vector3d>> points3D,
        const std::vector<py::dict> camera_dicts,
        const std::vector<Eigen::Vector4d> rig_qvecs,
        const std::vector<Eigen::Vector3d> rig_tvecs,
        const double max_error_px
) {
    std::vector<Camera> cameras;
    for (size_t i = 0; i < camera_dicts.size(); ++i) {
        const auto& camera_dict = camera_dicts[i];
        Camera camera;
        camera.SetModelIdFromName(camera_dict["model"].cast<std::string>());
        camera.SetWidth(camera_dict["width"].cast<size_t>());
        camera.SetHeight(camera_dict["height"].cast<size_t>());
        camera.SetParams(camera_dict["params"].cast<std::vector<double>>());
        cameras.push_back(camera);
    }

    return rig_absolute_pose_estimation(
        points2D, points3D, cameras, rig_qvecs, rig_tvecs, max_error_px
    );
}
