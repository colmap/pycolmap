#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace py = pybind11;

#include <colmap/base/pose.h>

#include "absolute_pose.cc"
#include "generalized_absolute_pose.cc"
#include "essential_matrix.cc"
#include "fundamental_matrix.cc"
#include "homography_estimation.cc"
#include "homography_decomposition.cc"
#include "transformations.cc"
#include "sift.cc"
#include "pose_refinement.cc"
#include "two_view_geometry_estimation.cc"

#include "reconstruction/reconstruction.cc"

void init_reconstruction(py::module &);
void init_transforms(py::module &);

PYBIND11_MODULE(pycolmap, m) {
    m.doc() = "COLMAP plugin";
#ifdef VERSION_INFO
    m.attr("__version__") = py::str(VERSION_INFO);
#else
    m.attr("__version__") = py::str("dev");
#endif

    // Absolute pose.
    m.def("absolute_pose_estimation",
            &absolute_pose_estimation,
            py::arg("points2D"), py::arg("points3D"),
            py::arg("camera"),
            py::arg("max_error_px") = 12.0,
            py::arg("min_inlier_ratio") = 0.01,
            py::arg("min_num_trials") = 1000,
            py::arg("max_num_trials") = 100000,
            py::arg("confidence") = 0.9999,
            "Absolute pose estimation with non-linear refinement.");

    m.def("rig_absolute_pose_estimation", &rig_absolute_pose_estimation,
          py::arg("points2D"), py::arg("points3D"),
          py::arg("cameras"),
          py::arg("rig_qvecs"), py::arg("rig_tvecs"),
          py::arg("max_error_px") = 12.0,
          py::arg("min_inlier_ratio") = 0.01,
          py::arg("min_num_trials") = 1000,
          py::arg("max_num_trials") = 100000,
          py::arg("confidence") = 0.9999,
          "Absolute pose estimation with non-linear refinement.");

    // Essential matrix.
    m.def("essential_matrix_estimation",
          &essential_matrix_estimation,
          py::arg("points2D1"), py::arg("points2D2"),
          py::arg("camera1"), py::arg("camera2"),
          py::arg("max_error_px") = 4.0,
          py::arg("min_inlier_ratio") = 0.01,
          py::arg("min_num_trials") = 1000,
          py::arg("max_num_trials") = 100000,
          py::arg("confidence") = 0.9999,
          "LORANSAC + 5-point algorithm.");

    // Fundamental matrix.
    m.def("fundamental_matrix_estimation",
          &fundamental_matrix_estimation,
          py::arg("points2D1"), py::arg("points2D2"),
          py::arg("max_error_px") = 4.0,
          py::arg("min_inlier_ratio") = 0.01,
          py::arg("min_num_trials") = 1000,
          py::arg("max_num_trials") = 100000,
          py::arg("confidence") = 0.9999,
          "LORANSAC + 7-point algorithm.");
  
    // Generic two view geometry estimation.
    m.def("two_view_geometry_estimation", &two_view_geometry_estimation,
          py::arg("points2D1"),
          py::arg("points2D2"),
          py::arg("camera_dict1"),
          py::arg("camera_dict2"),
          py::arg("max_error_px") = 4.0,
          py::arg("min_inlier_ratio") = 0.01,
          py::arg("min_num_trials") = 1000,
          py::arg("max_num_trials") = 100000,
          py::arg("confidence") = 0.9999,
          "Generic two-view geometry estimation");

    // Homography matrix estimation.
    m.def("homography_matrix_estimation", &homography_matrix_estimation,
          py::arg("points2D1"), py::arg("points2D2"),
          py::arg("max_error_px") = 4.0,
          py::arg("min_inlier_ratio") = 0.01,
          py::arg("min_num_trials") = 1000,
          py::arg("max_num_trials") = 100000,
          py::arg("confidence") = 0.9999,
          "LORANSAC + 4-point DLT algorithm.");
    
    // Homography Decomposition.
    m.def("homography_decomposition", &homography_decomposition_estimation,
          py::arg("H"),
          py::arg("K1"),
          py::arg("K2"),
          py::arg("points1"),
          py::arg("points2"),
          "Analytical Homography Decomposition.");

    // SIFT.
    m.def("extract_sift", &extract_sift,
          py::arg("image"),
          py::arg("num_octaves") = 4, py::arg("octave_resolution") = 3, py::arg("first_octave") = 0,
          py::arg("edge_thresh") = 10.0, py::arg("peak_thresh") = 0.01, py::arg("upright") = false,
          "Extract SIFT features.");

    // Standalone Pose Refinement
    m.def("pose_refinement", &pose_refinement,
          py::arg("tvec"), py::arg("qvec"),
          py::arg("points2D"), py::arg("points3D"),
          py::arg("inlier_mask"),
          py::arg("camera"),
          "Non-linear refinement.");

    // Reconstruction bindings
    init_reconstruction(m);

    // Automatic conversion from python dicts to colmap cameras for backwards compatibility
    py::implicitly_convertible<py::dict, colmap::Camera>();
    // Transformation Bindings
    init_transforms(m);
}
