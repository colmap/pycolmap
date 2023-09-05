// Author: Philipp Lindenberger (Phil26AT)

#include "colmap/geometry/sim3.h"
#include "colmap/image/warp.h"
#include "colmap/camera/models.h"

using namespace colmap;

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

namespace py = pybind11;

#include "log_exceptions.h"


void init_transforms(py::module& m) {
    m.def("qvec_to_rotmat", &colmap::QuaternionToRotationMatrix,
          py::arg("qvec"),
          "Convert COLMAP quaternion to rotation matrix");
    m.def("rotmat_to_qvec", &colmap::RotationMatrixToQuaternion,
          py::arg("rotmat"),
          "Convert rotation matrix to colmap quaternion");
    m.def("qvec_rotate_point", &colmap::QuaternionRotatePoint,
          py::arg("qvec"),
          py::arg("xyz"),
          "Rotate world point");
    m.def("concat_quat", &colmap::ConcatenateQuaternions,
          py::arg("qvec1"),
          py::arg("qvec2"),
          "Concatenate Quaternion rotations such that the rotation of qvec1 is applied"
          "before the rotation of qvec2.");
    m.def("invert_qvec", &colmap::InvertQuaternion,
          py::arg("qvec"),
          "Returns inverted qvec");
    m.def("normalize_qvec", &colmap::InvertQuaternion,
          py::arg("qvec"),
          "Returns normalized qvec");

    m.def("projection_center_from_pose", &colmap::ProjectionCenterFromPose,
          py::arg("qvec"), py::arg("tvec"),
          "Extract camera projection center from projection parameters.");
    m.def("projection_center_from_matrix", &colmap::ProjectionCenterFromMatrix,
          py::arg("proj_matrix"),
          "Extract camera projection center from projection matrix, "
          "i.e. the projection center in world coordinates `-R^T t`.");

    m.def("relative_pose", [](const Eigen::Vector4d& qvec1,
                         const Eigen::Vector3d& tvec1,
                         const Eigen::Vector4d& qvec2,
                         const Eigen::Vector3d& tvec2) {
            Eigen::Vector4d qvec12;
            Eigen::Vector3d tvec12;
            colmap::ComputeRelativePose(
                qvec1, tvec1, qvec2, tvec2, &qvec12, &tvec12);
            return std::make_pair(qvec12, tvec12);
        },
        py::arg("qvec1"), py::arg("tvec1"), py::arg("qvec2"), py::arg("tvec2"));

    m.def("concatenate_poses", [](const Eigen::Vector4d& qvec1,
                         const Eigen::Vector3d& tvec1,
                         const Eigen::Vector4d& qvec2,
                         const Eigen::Vector3d& tvec2) {
            Eigen::Vector4d qvec12;
            Eigen::Vector3d tvec12;
            colmap::ConcatenatePoses(
                qvec1, tvec1, qvec2, tvec2, &qvec12, &tvec12);
            return std::make_pair(qvec12, tvec12);
        },
        py::arg("qvec1"), py::arg("tvec1"), py::arg("qvec2"), py::arg("tvec2"));
}
