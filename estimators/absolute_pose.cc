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

#include <iostream>
#include <fstream>

#include "colmap/base/camera.h"
#include "colmap/estimators/pose.h"
#include "colmap/util/random.h"

using namespace colmap;

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace py = pybind11;

#include "log_exceptions.h"
#include "helpers.h"


py::dict absolute_pose_estimation(
        const std::vector<Eigen::Vector2d> points2D,
        const std::vector<Eigen::Vector3d> points3D,
        Camera& camera,
        const AbsolutePoseEstimationOptions estimation_options,
        const AbsolutePoseRefinementOptions refinement_options
) {
    SetPRNGSeed(0);

    // Check that both vectors have the same size.
    THROW_CHECK_EQ(points2D.size(), points3D.size());

    // Failure output dictionary.
    py::dict failure_dict;
    failure_dict["success"] = false;
    //
    // Absolute pose estimation.
    Eigen::Vector4d qvec;
    Eigen::Vector3d tvec;
    size_t num_inliers;
    std::vector<char> inlier_mask;

    if (!EstimateAbsolutePose(estimation_options, points2D, points3D, &qvec, &tvec, &camera, &num_inliers, &inlier_mask)) {
        return failure_dict;
    }

    // Absolute pose refinement.
    if (!RefineAbsolutePose(refinement_options, inlier_mask, points2D, points3D, &qvec, &tvec, &camera)) {
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

    // Success output dictionary.
    py::dict success_dict;
    success_dict["success"] = true;
    success_dict["qvec"] = qvec;
    success_dict["tvec"] = tvec;
    success_dict["num_inliers"] = num_inliers;
    success_dict["inliers"] = inliers;

    return success_dict;
}

py::dict absolute_pose_estimation(
        const std::vector<Eigen::Vector2d> points2D,
        const std::vector<Eigen::Vector3d> points3D,
        Camera& camera,
        const double max_error_px,
        const double min_inlier_ratio,
        const int min_num_trials,
        const int max_num_trials,
        const double confidence
) {
    // Absolute pose estimation parameters.
    AbsolutePoseEstimationOptions abs_pose_options;
    abs_pose_options.estimate_focal_length = false;
    abs_pose_options.ransac_options.max_error = max_error_px;
    abs_pose_options.ransac_options.min_inlier_ratio = min_inlier_ratio;
    abs_pose_options.ransac_options.min_num_trials = min_num_trials;
    abs_pose_options.ransac_options.max_num_trials = max_num_trials;
    abs_pose_options.ransac_options.confidence = confidence;

    // Refine absolute pose parameters.
    AbsolutePoseRefinementOptions abs_pose_refinement_options;
    abs_pose_refinement_options.refine_focal_length = false;
    abs_pose_refinement_options.refine_extra_params = false;
    abs_pose_refinement_options.print_summary = false;

    return absolute_pose_estimation(
            points2D, points3D, camera,
            abs_pose_options, abs_pose_refinement_options);
}


py::dict pose_refinement(
        const Eigen::Vector3d tvec,
        const Eigen::Vector4d qvec,
        const std::vector<Eigen::Vector2d> points2D,
        const std::vector<Eigen::Vector3d> points3D,
        const std::vector<bool> inlier_mask,
        const Camera camera,
        const AbsolutePoseRefinementOptions refinement_options
) {
    SetPRNGSeed(0);

    // Check that both vectors have the same size.
    THROW_CHECK_EQ(points2D.size(), points3D.size());
    THROW_CHECK_EQ(inlier_mask.size(), points2D.size());

    // Failure output dictionary.
    py::dict failure_dict;
    failure_dict["success"] = false;

    // Absolute pose estimation.
    Eigen::Vector4d qvec_refined = qvec;
    Eigen::Vector3d tvec_refined = tvec;
    std::vector<char> inlier_mask_char;
    for (size_t i = 0; i < inlier_mask.size(); ++i) {
        if(inlier_mask[i])
        {
            inlier_mask_char.emplace_back(1);
        }
        else
        {
            inlier_mask_char.emplace_back(0);
        }
    }

    // Absolute pose refinement.
    if (!RefineAbsolutePose(
            refinement_options, inlier_mask_char,
            points2D, points3D, &qvec_refined, &tvec_refined,
            const_cast<Camera*>(&camera))) {
        return failure_dict;
    }

    // Success output dictionary.
    py::dict success_dict;
    success_dict["success"] = true;
    success_dict["qvec"] = qvec_refined;
    success_dict["tvec"] = tvec_refined;

    return success_dict;
}


void bind_absolute_pose_estimation(py::module& m) {
    auto PyEstimationOptions =
        py::class_<AbsolutePoseEstimationOptions>(m, "AbsolutePoseEstimationOptions")
        .def(py::init<>([]() {
            AbsolutePoseEstimationOptions options;
            options.estimate_focal_length = false;
            return options;
        }))
        .def_readwrite("estimate_focal_length", &AbsolutePoseEstimationOptions::estimate_focal_length)
        .def_readwrite("num_focal_length_samples", &AbsolutePoseEstimationOptions::num_focal_length_samples)
        .def_readwrite("min_focal_length_ratio", &AbsolutePoseEstimationOptions::min_focal_length_ratio)
        .def_readwrite("max_focal_length_ratio", &AbsolutePoseEstimationOptions::max_focal_length_ratio)
        .def_readwrite("ransac", &AbsolutePoseEstimationOptions::ransac_options);
    make_dataclass(PyEstimationOptions);

    auto PyRefinementOptions =
        py::class_<AbsolutePoseRefinementOptions>(m, "AbsolutePoseRefinementOptions")
        .def(py::init<>([]() {
            AbsolutePoseRefinementOptions options;
            options.refine_focal_length = false;
            options.refine_extra_params = false;
            options.print_summary = false;
            return options;
        }))
        .def_readwrite("gradient_tolerance", &AbsolutePoseRefinementOptions::gradient_tolerance)
        .def_readwrite("max_num_iterations", &AbsolutePoseRefinementOptions::max_num_iterations)
        .def_readwrite("loss_function_scale", &AbsolutePoseRefinementOptions::loss_function_scale)
        .def_readwrite("refine_focal_length", &AbsolutePoseRefinementOptions::refine_focal_length)
        .def_readwrite("refine_extra_params", &AbsolutePoseRefinementOptions::refine_extra_params)
        .def_readwrite("print_summary", &AbsolutePoseRefinementOptions::print_summary);
    make_dataclass(PyRefinementOptions);

    auto est_options = PyEstimationOptions().cast<AbsolutePoseEstimationOptions>();
    auto ref_options = PyRefinementOptions().cast<AbsolutePoseRefinementOptions>();

    m.def(
        "absolute_pose_estimation",
        static_cast<py::dict (*)(const std::vector<Eigen::Vector2d>,
                                 const std::vector<Eigen::Vector3d>,
                                 Camera&,
                                 const AbsolutePoseEstimationOptions,
                                 const AbsolutePoseRefinementOptions
                                 )>(&absolute_pose_estimation),
        py::arg("points2D"), py::arg("points3D"), py::arg("camera"),
        py::arg("estimation_options") = est_options,
        py::arg("refinement_options") = ref_options,
        "Absolute pose estimation with non-linear refinement.");

    m.def(
        "absolute_pose_estimation",
        static_cast<py::dict (*)(const std::vector<Eigen::Vector2d>,
                                 const std::vector<Eigen::Vector3d>,
                                 Camera&,
                                 const double, const double,
                                 const int, const int, const double
                                 )>(&absolute_pose_estimation),
        py::arg("points2D"), py::arg("points3D"), py::arg("camera"),
        py::arg("max_error_px") = est_options.ransac_options.max_error,
        py::arg("min_inlier_ratio") = est_options.ransac_options.min_inlier_ratio,
        py::arg("min_num_trials") = est_options.ransac_options.min_num_trials,
        py::arg("max_num_trials") = est_options.ransac_options.max_num_trials,
        py::arg("confidence") = est_options.ransac_options.confidence,
        "Absolute pose estimation with non-linear refinement.");

    m.def(
        "pose_refinement", &pose_refinement,
        py::arg("tvec"), py::arg("qvec"),
        py::arg("points2D"), py::arg("points3D"),
        py::arg("inlier_mask"),
        py::arg("camera"),
        py::arg("refinement_options") = ref_options,
        "Non-linear refinement.");
}
