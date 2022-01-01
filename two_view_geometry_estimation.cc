
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

/*
* Estimate two-view geometry relationship for a calibrated camera.
* 
* param points2D1,
* param points2D2,
* param camera_dict1
* param camera_dict2
* param max_error_px,
* param min_inlier_ratio,
* param min_num_trials,
* param max_num_trials,
* param confidence
* @return The most probable rotation matrix (3x3), translation vector (3x1), and two view
*         configuration type (UNDEFINED, DEGENERATE, CALIBRATED, UNCALIBRATED, PLANAR
*         PANORAMIC, PLANAR_OR_PANORAMIC, WATERMARK, or MULTIPLE).
*         See https://github.com/colmap/colmap/blob/dev/src/estimators/two_view_geometry.h#L48
*/
py::dict two_view_geometry_estimation(
        const std::vector<Eigen::Vector2d> points2D1,
        const std::vector<Eigen::Vector2d> points2D2,
        const py::dict camera_dict1,
        const py::dict camera_dict2,
        const double max_error_px,
        const double min_inlier_ratio,
        const int min_num_trials,
        const int max_num_trials,
        const double confidence
) {
    SetPRNGSeed(0);

    // Check that both vectors have the same size.
    assert(points2D1.size() == points2D2.size());

    // Failure output dictionary.
    py::dict failure_dict;
    failure_dict["success"] = false;

    // Create cameras.
    Camera camera1;
    camera1.SetModelIdFromName(camera_dict1["model"].cast<std::string>());
    camera1.SetWidth(camera_dict1["width"].cast<size_t>());
    camera1.SetHeight(camera_dict1["height"].cast<size_t>());
    camera1.SetParams(camera_dict1["params"].cast<std::vector<double>>());

    Camera camera2;
    camera2.SetModelIdFromName(camera_dict2["model"].cast<std::string>());
    camera2.SetWidth(camera_dict2["width"].cast<size_t>());
    camera2.SetHeight(camera_dict2["height"].cast<size_t>());
    camera2.SetParams(camera_dict2["params"].cast<std::vector<double>>());

    const FeatureMatches matches;
    const Options options;
 
    TwoViewGeometry two_view_geometry;
    TwoViewGeometry::Options two_view_geometry_options;
    two_view_geometry_options.ransac_options.min_num_trials = 30;
    two_view_geometry_options.ransac_options.max_error = options.init_max_error;
    two_view_geometry.EstimateCalibrated(camera1, points1, camera2, points2, matches, two_view_geometry_options);

    // Success output dictionary.
    py::dict success_dict;

    if (!two_view_geometry.EstimateRelativePose(camera1, points1, camera2, points2)) {
        return failure_dict;
    } else {
        success_dict["success"] = true;
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
  
    success_dict["configuration_type"] = two_view_geometry.config;
    success_dict["qvec"] = two_view_geometry.qvec;
    success_dict["tvec"] = two_view_geometry.tvec;
    success_dict["num_inliers"] = num_inliers;
    success_dict["inliers"] = inliers;
    
    return success_dict;
}
