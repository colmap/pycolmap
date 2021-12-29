#pragma once
#include "colmap/base/camera_models.h"
#include "colmap/base/projection.h"
#include "colmap/base/reconstruction.h"
#include "colmap/util/misc.h"
#include "colmap/util/ply.h"
#include "colmap/util/types.h"

using namespace colmap;

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;
using namespace pybind11::literals;

#include "log_exceptions.h"

SimilarityTransform3 AlignPosesBetweenReconstructions(Reconstruction& self,
                                                      const Reconstruction& ref,
                                                      const double min_inlier_observations,
                                                      double max_reproj_error);

SimilarityTransform3 AlignPointsBetweenReconstructions(Reconstruction& self,
                                                       const Reconstruction& ref, int min_overlap,
                                                       double max_error, double min_inlier_ratio);

SimilarityTransform3 AlignReconstruction(Reconstruction& self,
                                         const std::vector<std::string>& image_names,
                                         const std::vector<Eigen::Vector3d>& locations,
                                         const int min_common_images);

SimilarityTransform3 RobustAlignReconstruction(Reconstruction& self,
                                               const std::vector<std::string>& image_names,
                                               const std::vector<Eigen::Vector3d>& locations,
                                               const int min_common_images, double max_error,
                                               double min_inlier_ratio);

void init_reconstruction_utils(py::module& m);

template <bool kEstimateScale>
inline bool ComputeRobustAlignmentBetweenPoints(const std::vector<Eigen::Vector3d>& src,
                                                const std::vector<Eigen::Vector3d>& dst,
                                                double max_error, double min_inlier_ratio,
                                                Eigen::Matrix3x4d* alignment) {
    RANSACOptions ransac_options;
    ransac_options.max_error = max_error;
    ransac_options.min_inlier_ratio = min_inlier_ratio;

    LORANSAC<SimilarityTransformEstimator<3, kEstimateScale>,
             SimilarityTransformEstimator<3, kEstimateScale>>
        ransac(ransac_options);

    const auto report = ransac.Estimate(src, dst);

    if (report.success) {
        *alignment = report.model;
    }
    return report.success;
}