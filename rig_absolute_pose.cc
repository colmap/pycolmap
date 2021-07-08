// Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch at inf.ethz.ch)

#include <iostream>
#include <fstream>

#include "colmap/optim/bundle_adjustment.h"
#include "colmap/base/camera.h"
#include "colmap/base/projection.h"
#include "colmap/base/cost_functions.h"
#include "colmap/base/similarity_transform.h"
#include "colmap/estimators/generalized_absolute_pose.h"
#include "colmap/estimators/pose.h"
#include "colmap/optim/ransac.h"
#include "colmap/util/random.h"
#include "colmap/util/matrix.h"
#include "colmap/util/misc.h"
#include "colmap/base/camera_models.h"

#ifdef OPENMP_ENABLED
#include <omp.h>
#endif

using namespace colmap;

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace py = pybind11;


py::dict rig_absolute_pose_estimation(
        const std::vector<Eigen::Vector3d> tvec,
        const std::vector<Eigen::Vector4d> qvec,
        const std::vector<std::vector<Eigen::Vector2d>> points2D,
        const std::vector<std::vector<Eigen::Vector3d>> points3D,
        const std::vector<py::dict> cameras_dict,
        const double max_error // cosine distance (see https://github.com/colmap/colmap/blob/ff9a463067a2656d1f59d12109fe2931e29e3ca0/src/estimators/generalized_absolute_pose.cc#L319), squared, default value  = 1e-5 from https://github.com/colmap/colmap/blob/c423ca974459273029c770010bb78a7f0f59d6c7/src/estimators/generalized_absolute_pose_test.cc
) {
    SetPRNGSeed(0);
    Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");
    const size_t kNumTforms = cameras_dict.size();

    // Check that vectors have the same size.
    assert(points2D.size() == kNumTforms);
    assert(points3D.size() == kNumTforms);
    for (size_t i = 0; i < kNumTforms; ++i) {
        assert(points2D[i].size() == points3D[i].size());
    }
    assert(tvec.size() == kNumTforms);
    assert(qvec.size() == kNumTforms);

    std::vector<Eigen::Matrix3x4d> rel_tforms;
    std::vector<Eigen::Vector3d> tvec_copy;
    std::vector<Eigen::Vector4d> qvec_copy;
    for (size_t i = 0; i < kNumTforms; ++i) {
        qvec_copy.emplace_back(NormalizeQuaternion(qvec[i]));
        tvec_copy.emplace_back(tvec[i]);
        rel_tforms.emplace_back(ComposeProjectionMatrix(qvec_copy[i], tvec_copy[i]));
    }

    // Failure output dictionary.
    py::dict failure_dict;
    failure_dict["success"] = false;

    // Create camera.
    std::vector<Camera> cameras;
    for (size_t i = 0; i < kNumTforms; ++i) {
        cameras.emplace_back();
        cameras.back().SetModelIdFromName(cameras_dict[i]["model"].cast<std::string>());
        cameras.back().SetWidth(cameras_dict[i]["width"].cast<size_t>());
        cameras.back().SetHeight(cameras_dict[i]["height"].cast<size_t>());
        cameras.back().SetParams(cameras_dict[i]["params"].cast<std::vector<double>>());
    }

    std::vector<GP3PEstimator::X_t> flatten_points2D;
    std::vector<Eigen::Vector3d> flatten_points3D;
    for (size_t i = 0; i < kNumTforms; ++i) {
        for (size_t j = 0; j < points2D[i].size(); ++j) {
            // Eigen::Vector3d point3D_camera = points3D[i];
            // orig_tforms[i % kNumTforms].TransformPoint(&point3D_camera);
            flatten_points2D.emplace_back();
            flatten_points2D.back().rel_tform = rel_tforms[i];
            flatten_points2D.back().xy = cameras[i].ImageToWorld(points2D[i][j]); //point3D_camera.hnormalized();
            flatten_points3D.emplace_back(points3D[i][j]);
        }
    }

    // Absolute pose estimation parameters.
    RANSACOptions options;
    options.max_error = max_error;
    options.min_inlier_ratio = 0.01;
    options.min_num_trials = 1000;
    options.max_num_trials = 100000;
    options.confidence = 0.9999;

    // Absolute pose estimation.
    RANSAC<GP3PEstimator> ransac(options);
    const auto report = ransac.Estimate(flatten_points2D, flatten_points3D);
    if (!report.success)
    {
        return failure_dict;
    }

    Eigen::Matrix3x4d absolute_pose_rig = report.model;
    Eigen::Vector4d absolute_pose_rig_qvec = NormalizeQuaternion(RotationMatrixToQuaternion(absolute_pose_rig.leftCols<3>()));
    Eigen::Vector3d  absolute_pose_rig_tvec = absolute_pose_rig.rightCols<1>();
    size_t num_inliers = report.support.num_inliers;
    std::vector<char> inlier_mask = report.inlier_mask;
    std::cout << std::endl;
    std::cout << "pose =" << absolute_pose_rig_qvec.format(CommaInitFmt) << absolute_pose_rig_tvec.format(CommaInitFmt) 
              << " --- inliers: " << num_inliers << " / " << inlier_mask.size() << std::endl;
    
    assert(inlier_mask.size() == flatten_points2D.size());

    AbsolutePoseRefinementOptions abs_pose_refinement_options;
    abs_pose_refinement_options.refine_focal_length = false;
    abs_pose_refinement_options.refine_extra_params = false;

    ceres::LossFunction* loss_function = new ceres::CauchyLoss(abs_pose_refinement_options.loss_function_scale);
    ceres::Problem problem;
    std::unordered_set<double*> parameterized_qvec_data;

    size_t index = 0;
    double* rig_qvec_data = absolute_pose_rig_qvec.data();
    double* rig_tvec_data = absolute_pose_rig_tvec.data();
    for (size_t i = 0; i < kNumTforms; ++i) {
        double* qvec_data = qvec_copy[i].data();
        double* tvec_data = tvec_copy[i].data();
        double* camera_params_data = cameras[i].ParamsData();
        size_t num_observations = 0;
        for (size_t j = 0; j < points2D[i].size(); ++j) {
            size_t local_index = index;
            index += 1;
            if (!inlier_mask[local_index]) {
                continue;
            }
            num_observations += 1;
            ceres::CostFunction* cost_function = nullptr;
            switch (cameras[i].ModelId()) {
#define CAMERA_MODEL_CASE(CameraModel)                                      \
            case CameraModel::kModelId:                                               \
                cost_function =                                                         \
                    RigBundleAdjustmentCostFunction<CameraModel>::Create(points2D[i][j]); \
                break;

            CAMERA_MODEL_SWITCH_CASES

#undef CAMERA_MODEL_CASE
            }
            problem.AddResidualBlock(cost_function, loss_function, rig_qvec_data,
                                    rig_tvec_data, qvec_data, tvec_data,
                                    flatten_points3D[local_index].data(), camera_params_data);
            problem.SetParameterBlockConstant(flatten_points3D[local_index].data());
        }

        if (num_observations > 0)
        {
            // Quaternion parameterization.
            parameterized_qvec_data.insert(qvec_data);
            parameterized_qvec_data.insert(rig_qvec_data);

            // Camera parameterization.
            problem.SetParameterBlockConstant(camera_params_data);

            // Set the relative pose of the camera constant
            problem.SetParameterBlockConstant(qvec_data);
            problem.SetParameterBlockConstant(tvec_data);
        }
    }

    // Apply Quaternion parameterization uniquely rig_qvec_data appears in multiple loops.
    for (double* qvec_data : parameterized_qvec_data) {
    ceres::LocalParameterization* quaternion_parameterization =
        new ceres::QuaternionParameterization;
    problem.SetParameterization(qvec_data, quaternion_parameterization);
    }

    ceres::Solver::Options solver_options;
    solver_options.gradient_tolerance = abs_pose_refinement_options.gradient_tolerance;
    solver_options.max_num_iterations = abs_pose_refinement_options.max_num_iterations;
    solver_options.linear_solver_type = ceres::DENSE_QR;
    
    // The overhead of creating threads is too large.
    solver_options.num_threads = 1;
#if CERES_VERSION_MAJOR < 2
    solver_options.num_linear_solver_threads = 1;
#endif  // CERES_VERSION_MAJOR

    ceres::Solver::Summary summary;
    ceres::Solve(solver_options, &problem, &summary);

    PrintHeading2("Pose refinement report");
    PrintSolverSummary(summary);
    std::cout << "pose =" << absolute_pose_rig_qvec.format(CommaInitFmt) << absolute_pose_rig_tvec.format(CommaInitFmt)
              << " --- bundle: success=" << summary.IsSolutionUsable() << std::endl;
    
    if (!summary.IsSolutionUsable())
    {
        return failure_dict;
    }

    // Convert vector<char> to vector<bool>.
    std::vector<bool> inliers;
    for (auto it : inlier_mask) {
        if (it) {
            inliers.push_back(true);
        } else {
            inliers.push_back(false);
        }
    }

    /// Success output dictionary.
    py::dict success_dict;
    success_dict["success"] = true;
    success_dict["qvec"] = absolute_pose_rig_qvec;
    success_dict["tvec"] = absolute_pose_rig_tvec;
    success_dict["num_inliers"] = num_inliers;
    success_dict["inliers"] = inliers;
    
    return success_dict;
}
