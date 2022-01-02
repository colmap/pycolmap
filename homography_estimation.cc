#include <iostream>
#include <fstream>

#include "colmap/estimators/homography_matrix.h"
#include "colmap/optim/loransac.h"

using namespace colmap;

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace py = pybind11;

/**
 * Recover the most probable pose from the inputted homography matrix.
 * 
 * @param points2D1 First set of corresponding points.
 * @param points2D2 Second set of corresponding points.
 * @param max_error_px
 * @param min_inlier_ratio
 * @param min_num_trials
 * @param max_num_trials
 * @return The estimated homography matrix (3x3), ...
 */
py::dict homography_matrix_estimation(
        const std::vector<Eigen::Vector2d> points2D1,
        const std::vector<Eigen::Vector2d> points2D2,
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

    // Fundamental matrix estimation parameters.
    RANSACOptions ransac_options;
    ransac_options.max_error = max_error_px;
    ransac_options.min_inlier_ratio = min_inlier_ratio;
    ransac_options.min_num_trials = min_num_trials;
    ransac_options.max_num_trials = max_num_trials;
    ransac_options.confidence = confidence;
    
    // Estimate planar or panoramic model.
    LORANSAC<
        HomographyMatrixEstimator,
        HomographyMatrixEstimator
    > H_ransac(ransac_options);

    // Homography matrix estimation.
    const auto report = H_ransac.Estimate(points2D1, points2D2);

    if (!report.success) {
        return failure_dict;
    }

    // Recover data from report.
    const Eigen::Matrix3d H = report.model;
    const size_t num_inliers = report.support.num_inliers;
    const auto inlier_mask = report.inlier_mask;
    
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
    py::dict success_dict;
    success_dict["success"] = true;
    success_dict["H"] = H;
    success_dict["num_inliers"] = num_inliers;
    success_dict["inliers"] = inliers;
    
    return success_dict;
}
