// Authors: Mihai-Dusmanu (mihaidusmanu), Paul-Edouard Sarlin (skydes)

#include <iostream>
#include <fstream>

#include "colmap/base/camera.h"
#include "colmap/base/essential_matrix.h"
#include "colmap/base/pose.h"
#include "colmap/estimators/essential_matrix.h"
#include "colmap/optim/loransac.h"
#include "colmap/util/random.h"

using namespace colmap;

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace py = pybind11;

#include "log_exceptions.h"


py::dict essential_matrix_estimation(
        const std::vector<Eigen::Vector2d> points2D1,
        const std::vector<Eigen::Vector2d> points2D2,
        Camera& camera1,
        Camera& camera2,
        const RANSACOptions options
) {
    SetPRNGSeed(0);

    // Check that both vectors have the same size.
    THROW_CHECK_EQ(points2D1.size(), points2D2.size());

    // Failure output dictionary.
    py::dict failure_dict;
    failure_dict["success"] = false;
    py::gil_scoped_release release;

    // Image to world.
    std::vector<Eigen::Vector2d> world_points2D1;
    for (size_t idx = 0; idx < points2D1.size(); ++idx) {
        world_points2D1.push_back(camera1.ImageToWorld(points2D1[idx]));
    }

    std::vector<Eigen::Vector2d> world_points2D2;
    for (size_t idx = 0; idx < points2D2.size(); ++idx) {
        world_points2D2.push_back(camera2.ImageToWorld(points2D2[idx]));
    }

    // Compute world error.
    const double max_error_px = options.max_error;
    const double max_error = 0.5 * (
        max_error_px / camera1.MeanFocalLength() + max_error_px / camera2.MeanFocalLength()
    );
    RANSACOptions ransac_options(options);
    ransac_options.max_error = max_error;

    LORANSAC<
        EssentialMatrixFivePointEstimator,
        EssentialMatrixFivePointEstimator
    > ransac(ransac_options);

    // Essential matrix estimation.
    const auto report = ransac.Estimate(world_points2D1, world_points2D2);

    if (!report.success) {
        return failure_dict;
    }

    // Recover data from report.
    const Eigen::Matrix3d E = report.model;
    const size_t num_inliers = report.support.num_inliers;
    const auto inlier_mask = report.inlier_mask;

    // Pose from essential matrix.
    std::vector<Eigen::Vector2d> inlier_world_points2D1;
    std::vector<Eigen::Vector2d> inlier_world_points2D2;

    for (size_t idx = 0; idx < inlier_mask.size(); ++idx) {
        if (inlier_mask[idx]) {
            inlier_world_points2D1.push_back(world_points2D1[idx]);
            inlier_world_points2D2.push_back(world_points2D2[idx]);
        }
    }

    Eigen::Matrix3d R;
    Eigen::Vector3d tvec;
    std::vector<Eigen::Vector3d> points3D;
    PoseFromEssentialMatrix(E, inlier_world_points2D1, inlier_world_points2D2, &R, &tvec, &points3D);

    Eigen::Vector4d qvec = RotationMatrixToQuaternion(R);

    // Convert vector<char> to vector<int>.
    std::vector<bool> inliers;
    for (auto it : inlier_mask) {
        if (it) {
            inliers.push_back(true);
        } else {
            inliers.push_back(false);
        }
    }

    // Success output dictionary.
    py::gil_scoped_acquire acquire;
    py::dict success_dict;
    success_dict["success"] = true;
    success_dict["E"] = E;
    success_dict["qvec"] = qvec;
    success_dict["tvec"] = tvec;
    success_dict["num_inliers"] = num_inliers;
    success_dict["inliers"] = inliers;

    return success_dict;
}

py::dict essential_matrix_estimation(
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
    RANSACOptions ransac_options;
    ransac_options.max_error = max_error_px;
    ransac_options.min_inlier_ratio = min_inlier_ratio;
    ransac_options.min_num_trials = min_num_trials;
    ransac_options.max_num_trials = max_num_trials;
    ransac_options.confidence = confidence;
    return essential_matrix_estimation(
            points2D1, points2D2, camera1, camera2, ransac_options);
}

void bind_essential_matrix_estimation(py::module& m) {
    auto est_options = m.attr("RANSACOptions")().cast<RANSACOptions>();

    m.def(
        "essential_matrix_estimation",
        static_cast<py::dict (*)(const std::vector<Eigen::Vector2d>,
                                 const std::vector<Eigen::Vector2d>,
                                 Camera&, Camera&,
                                 const RANSACOptions
                                 )>(&essential_matrix_estimation),
        py::arg("points2D1"), py::arg("points2D2"),
        py::arg("camera1"), py::arg("camera2"),
        py::arg("estimation_options") = est_options,
        "LORANSAC + 5-point algorithm.");

    m.def(
        "essential_matrix_estimation",
        static_cast<py::dict (*)(const std::vector<Eigen::Vector2d>,
                                 const std::vector<Eigen::Vector2d>,
                                 Camera&, Camera&,
                                 const double, const double,
                                 const int, const int, const double
                                 )>(&essential_matrix_estimation),
        py::arg("points2D1"), py::arg("points2D2"),
        py::arg("camera1"), py::arg("camera2"),
        py::arg("max_error_px") = est_options.max_error,
        py::arg("min_inlier_ratio") = est_options.min_inlier_ratio,
        py::arg("min_num_trials") = est_options.min_num_trials,
        py::arg("max_num_trials") = est_options.max_num_trials,
        py::arg("confidence") = est_options.confidence,
        "LORANSAC + 5-point algorithm.");
}
