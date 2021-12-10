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
    m.def("invert_qvec", &colmap::InvertQuaternion,
          py::arg("qvec"),
          "Returns inverted qvec");
    m.def("normalize_qvec", &colmap::InvertQuaternion,
          py::arg("qvec"),
          "Returns normalized qvec");

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
