// Author: Philipp Lindenberger (Phil26AT)

#include "colmap/base/similarity_transform.h"
#include "colmap/base/warp.h"
#include "colmap/base/camera_models.h"

using namespace colmap;

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>

namespace py = pybind11;

#include "log_exceptions.h"


py::dict image_to_world(
        const std::vector<Eigen::Vector2d> points2D,
        const colmap::Camera& camera
) {
    static unsigned int m_warnings = 0;
    m_warnings++;
    if (m_warnings == 1) {
        std::cerr<<"DeprecationWarning: This function will "
        <<"be removed in a future release. "
        <<"Use camera.image_to_world instead."<<std::endl;
    }

    // Image to world.
    std::vector<Eigen::Vector2d> world_points2D;
    for (size_t idx = 0; idx < points2D.size(); ++idx) {
        world_points2D.push_back(camera.ImageToWorld(points2D[idx]));
    }

    // Mean focal length.
    const double mean_focal_length = camera.MeanFocalLength();

    // Success output dictionary.
    py::dict success_dict;
    success_dict["world_points"] = world_points2D;
    success_dict["mean_focal_length"] = mean_focal_length;

    return success_dict;
}

py::dict world_to_image(
        const std::vector<Eigen::Vector2d> world_points2D,
        const colmap::Camera& camera
) {
    static unsigned int m_warnings = 0;
    m_warnings++;
    if (m_warnings == 1) {
        std::cerr<<"DeprecationWarning: This function will "
        <<"be removed in a future release. "
        <<"Use camera.world_to_image instead."<<std::endl;
    }

    // World to image.
    std::vector<Eigen::Vector2d> image_points2D;
    for (size_t idx = 0; idx < world_points2D.size(); ++idx) {
        image_points2D.push_back(camera.WorldToImage(world_points2D[idx]));
    }

    // Success output dictionary.
    py::dict success_dict;
    success_dict["image_points"] = image_points2D;

    return success_dict;
}

void init_transforms(py::module& m) {
    m.def("compute_alignment",
        [](const Reconstruction& src_reconstruction,
            const Reconstruction& ref_reconstruction,
            const double min_inlier_observations,
            const double max_reproj_error) {
            THROW_CHECK_GE(min_inlier_observations, 0.0);
            THROW_CHECK_LE(min_inlier_observations, 1.0);
            Eigen::Matrix3x4d alignment;
            bool success =
                ComputeAlignmentBetweenReconstructions(src_reconstruction,
                    ref_reconstruction, min_inlier_observations,
                    max_reproj_error, &alignment);
            THROW_CHECK(success);
            return alignment;
        },
        py::arg("src_reconstruction").noconvert(),
        py::arg("ref_reconstruction").noconvert(),
        py::arg("min_inlier_observations") = 0.3,
        py::arg("max_reproj_error") = 8.0,
        py::keep_alive<1,2>(),
        py::keep_alive<1,3>());

    py::class_<SimilarityTransform3>(m, "SimilarityTransform3")
        .def(py::init<const Eigen::Matrix3x4d&>())
        .def(py::init<double, Eigen::Vector4d, Eigen::Vector3d>())
        .def_property_readonly_static("estimate", [](py::object){
            return py::cpp_function([](std::vector<Eigen::Vector3d> src,
                    std::vector<Eigen::Vector3d> dst){
                SimilarityTransform3 tform;
                bool success = tform.Estimate(src,dst);
                THROW_CHECK(success);
                return tform;
            });
        })
        .def_property_readonly("rotation", &SimilarityTransform3::Rotation)
        .def_property_readonly("translation", &SimilarityTransform3::Translation)
        .def_property_readonly("scale", &SimilarityTransform3::Scale)
        .def_property_readonly("matrix", &SimilarityTransform3::Matrix)
        .def("transform_point", [](const SimilarityTransform3& self,
                                   Eigen::Ref<Eigen::Vector3d> xyz){
            Eigen::Vector3d cpy(xyz);
            self.TransformPoint(&cpy);
            xyz = cpy;
        })
        .def("transform_pose", [](const SimilarityTransform3& self,
                                  Eigen::Ref<Eigen::Vector4d> qvec,
                                  Eigen::Ref<Eigen::Vector3d> tvec){
            Eigen::Vector3d cpyt(tvec);
            Eigen::Vector4d cpyq(qvec);
            self.TransformPose(&cpyq, &cpyt);
            qvec = cpyq;
            tvec = cpyt;
        })
        .def("inverse", &SimilarityTransform3::Inverse)
        .def("__repr__", [](const SimilarityTransform3& self){
            std::stringstream ss;
            ss<<"SimilarityTransform3:\n"
            <<self.Matrix();
            return ss.str();
        });

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
        
    m.def("world_to_image", [](int model_id,
        Eigen::Ref<Eigen::VectorXd> camera_params,
        Eigen::Ref<Eigen::Vector2d> uv,
        Eigen::Ref<Eigen::Vector2d> xy) {
        int num_params;
        switch (model_id) {
            #define CAMERA_MODEL_CASE(CameraModel)                             \
            case CameraModel::kModelId:                                        \
                num_params = CameraModel::kNumParams;                          \
                THROW_CHECK_EQ(num_params,camera_params.size());               \
                CameraModel::WorldToImage<double>(                             \
                    camera_params.data(), uv(0), uv(1),                        \
                    xy.data(), xy.data()+1);                                   \
                break;
            CAMERA_MODEL_SWITCH_CASES
            #undef CAMERA_MODEL_CASE
        }
    }, py::arg("model_id"),
       py::arg("camera_params").noconvert(),
       py::arg("uv").noconvert(),
       py::arg("xy").noconvert());

    // Image-to-world and world-to-image.
    m.def("image_to_world", &image_to_world, "Image to world transformation.");
    m.def("world_to_image", &world_to_image, "World to image transformation.");

    m.def("image_to_world", [](int model_id,
        Eigen::Ref<Eigen::VectorXd> camera_params,
        Eigen::Ref<Eigen::Vector2d> xy,
        Eigen::Ref<Eigen::Vector2d> uv) {
        int num_params;
        switch (model_id) {
            #define CAMERA_MODEL_CASE(CameraModel)                             \
            case CameraModel::kModelId:                                        \
                num_params = CameraModel::kNumParams;                          \
                THROW_CHECK_EQ(num_params,camera_params.size());               \
                CameraModel::ImageToWorld<double>(                             \
                    camera_params.data(), xy(0), xy(1),                        \
                    uv.data(), uv.data()+1);                                   \
                break;
            CAMERA_MODEL_SWITCH_CASES
            #undef CAMERA_MODEL_CASE
        }
    }, py::arg("model_id"),
       py::arg("camera_params").noconvert(),
       py::arg("xy").noconvert(),
       py::arg("uv").noconvert());

}
