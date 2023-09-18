// Authors: John Lambert (johnwlambert), Paul-Edouard Sarlin (skydes)

#include "colmap/estimators/homography_matrix.h"
#include "colmap/math/random.h"
#include "colmap/optim/loransac.h"

#include <fstream>
#include <iostream>

using namespace colmap;

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;

#include "log_exceptions.h"

py::dict homography_matrix_estimation(
    const std::vector<Eigen::Vector2d> points2D1,
    const std::vector<Eigen::Vector2d> points2D2,
    const RANSACOptions options) {
  SetPRNGSeed(0);

  // Check that both vectors have the same size.
  THROW_CHECK_EQ(points2D1.size(), points2D2.size());

  // Failure output dictionary.
  py::dict failure_dict;
  failure_dict["success"] = false;
  py::gil_scoped_release release;

  // Estimate planar or panoramic model.
  LORANSAC<HomographyMatrixEstimator, HomographyMatrixEstimator> H_ransac(
      options);

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
  py::gil_scoped_acquire acquire;
  py::dict success_dict;
  success_dict["success"] = true;
  success_dict["H"] = H;
  success_dict["num_inliers"] = num_inliers;
  success_dict["inliers"] = inliers;

  return success_dict;
}

void bind_homography_estimation(py::module& m) {
  auto est_options = m.attr("RANSACOptions")().cast<RANSACOptions>();

  m.def("homography_matrix_estimation",
        &homography_matrix_estimation,
        "points2D1"_a,
        "points2D2"_a,
        "estimation_options"_a = est_options,
        "LORANSAC + 4-point DLT algorithm.");
}
