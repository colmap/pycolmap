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

py::dict pose_refinement(
        const Eigen::Vector3d tvec,
        const Eigen::Vector4d qvec,
        const std::vector<Eigen::Vector2d> points2D,
        const std::vector<Eigen::Vector3d> points3D,
        const std::vector<bool> inlier_mask,
        const py::dict camera_dict
) {
    SetPRNGSeed(0);

    // Check that both vectors have the same size.
    assert(points2D.size() == points3D.size());
    assert(inlier_mask.size() == points2D.size());

    // Failure output dictionary.
    py::dict failure_dict;
    failure_dict["success"] = false;

    // Create camera.
    Camera camera;
    camera.SetModelIdFromName(camera_dict["model"].cast<std::string>());
    camera.SetWidth(camera_dict["width"].cast<size_t>());
    camera.SetHeight(camera_dict["height"].cast<size_t>());
    camera.SetParams(camera_dict["params"].cast<std::vector<double>>());

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

    // Refine absolute pose parameters.
    AbsolutePoseRefinementOptions abs_pose_refinement_options;
    abs_pose_refinement_options.refine_focal_length = false;
    abs_pose_refinement_options.refine_extra_params = false;
    abs_pose_refinement_options.print_summary = false;

    // Absolute pose refinement.
    if (!RefineAbsolutePose(abs_pose_refinement_options, inlier_mask_char, points2D, points3D, &qvec_refined, &tvec_refined, &camera)) {
        return failure_dict;
    }

    // Success output dictionary.
    py::dict success_dict;
    success_dict["success"] = true;
    success_dict["qvec"] = qvec_refined;
    success_dict["tvec"] = tvec_refined;
    
    return success_dict;
}
