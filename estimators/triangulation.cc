#include "colmap/estimators/triangulation.h"

#include "helpers.h"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>

using namespace colmap;
namespace py = pybind11;
using namespace pybind11::literals;
using TriPoseData = TriangulationEstimator::PoseData;

struct PoseData {
  PoseData(const Eigen::Matrix3x4d& proj_matrix_,
           const Eigen::Vector3d& pose_,
           const Camera& camera_)
      : proj_matrix(proj_matrix_), proj_center(pose_), camera(camera_) {}
  // The projection matrix for the image of the observation.
  Eigen::Matrix3x4d proj_matrix;
  // The projection center for the image of the observation.
  Eigen::Vector3d proj_center;
  // The camera for the image of the observation.
  const Camera& camera;
};

py::dict estimate_triangulation(
    const std::vector<TriangulationEstimator::PointData>& point_data,
    const std::vector<PoseData>& pose_data,
    const EstimateTriangulationOptions& tri_options) {
  py::dict failure_dict("success"_a = false);

  Eigen::Vector3d xyz;
  std::vector<char> inlier_mask;

  std::vector<TriPoseData> tri_pose_data;
  for (auto it = pose_data.cbegin(); it != pose_data.cend(); it++) {
    tri_pose_data.push_back(
        TriPoseData(it->proj_matrix, it->proj_center, &it->camera));
  }

  if (!EstimateTriangulation(
          tri_options, point_data, tri_pose_data, &inlier_mask, &xyz)) {
    return failure_dict;
  }

  // Convert vector<char> to vector<int>.
  std::vector<bool> inliers;
  for (auto it : inlier_mask) {
    if (it) {
      inliers.push_back(true);
    } else {
      inliers.push_back(false);
    }
  }

  py::dict success_dict(
      "success"_a = true, "point_3d"_a = xyz, "inliers"_a = inliers);
  return success_dict;
}

void bind_estimate_triangulation(py::module& m,
                                 py::class_<RANSACOptions> PyRANSACOptions) {
  py::class_<TriangulationEstimator::PointData>(m, "PointData")
      .def(py::init<const Eigen::Vector2d&, const Eigen::Vector2d&>());

  py::class_<PoseData>(m, "PoseData")
      .def(py::init<const Eigen::Matrix3x4d&,
                    const Eigen::Vector3d&,
                    const Camera&>());

  auto PyTriangulationOptions =
      py::class_<EstimateTriangulationOptions>(m,
                                               "EstimateTriangulationOptions")
          .def(py::init<>([PyRANSACOptions]() {
            EstimateTriangulationOptions options;
            // init through Python to obtain the new defaults defined in
            // __init__
            options.ransac_options = PyRANSACOptions().cast<RANSACOptions>();
            return options;
          }))
          .def_readwrite("min_tri_angle",
                         &EstimateTriangulationOptions::min_tri_angle);

  make_dataclass(PyTriangulationOptions);
  auto triangulation_options =
      PyTriangulationOptions().cast<EstimateTriangulationOptions>();

  m.def("estimate_triangulation",
        &estimate_triangulation,
        "point_data"_a,
        "pose_data"_a,
        "estimate_triangulation_opions"_a = triangulation_options,
        "Robustly estimate 3D point from observations in multiple views using "
        "RANSAC");
}