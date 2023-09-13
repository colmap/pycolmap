// Author: Philipp Lindenberger (Phil26AT)

#include "colmap/sensor/models.h"
#include "colmap/geometry/sim3.h"
#include "colmap/scene/reconstruction.h"
#include "colmap/estimators/alignment.h"
#include "colmap/estimators/similarity_transform.h"
#include "colmap/exe/model.h"
#include "colmap/optim/loransac.h"
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

inline Sim3d AlignReconstructionsWithPoses(const Reconstruction& src,
                                           const Reconstruction& tgt,
                                           const double min_inlier_observations,
                                           const double max_reproj_error) {
    THROW_CHECK_GE(min_inlier_observations, 0.0);
    THROW_CHECK_LE(min_inlier_observations, 1.0);
    Sim3d tgt_from_src;
    THROW_CHECK(AlignReconstructions(src, tgt, min_inlier_observations,
                                     max_reproj_error, &tgt_from_src));
    return tgt_from_src;
}

inline Sim3d AlignReconstructionsWithPoses(const Reconstruction& src,
                                           const Reconstruction& tgt,
                                           const double max_proj_center_error) {
    THROW_CHECK_GT(max_proj_center_error, 0.0);
    Sim3d tgt_from_src;
    THROW_CHECK(AlignReconstructions(src, tgt, max_proj_center_error, &tgt_from_src));
    return tgt_from_src;
}

inline Sim3d AlignReconstructionsWithPoints(const Reconstruction& src,
                                            const Reconstruction& tgt,
                                            const size_t min_common_observations,
                                            const double max_error,
                                            const double min_inlier_ratio) {
    THROW_CHECK_GT(min_common_observations, 0);
    THROW_CHECK_GT(max_error, 0.0);
    THROW_CHECK_GE(min_inlier_ratio, 0.0);
    THROW_CHECK_LE(min_inlier_ratio, 1.0);
    Sim3d tgt_from_src;
    THROW_CHECK(AlignReconstructionsViaPoints(src, tgt, min_common_observations,
                                              max_error, min_inlier_ratio, &tgt_from_src));
    return tgt_from_src;
}

inline Sim3d AlignReconstructionToLocationsWrapper(
        const Reconstruction& src,
        const std::vector<std::string>& image_names,
        const std::vector<Eigen::Vector3d>& locations,
        const int min_common_images,
        const RANSACOptions& ransac_options) {
    THROW_CHECK_GE(min_common_images, 3);
    THROW_CHECK_EQ(image_names.size(), locations.size());
    Sim3d locationsFromSrc;
    THROW_CHECK(AlignReconstructionToLocations(src, image_names, locations,
                                               min_common_images, ransac_options,
                                               &locationsFromSrc));
    return locationsFromSrc;
}

void bind_alignment(py::module& m) {
    py::class_<ImageAlignmentError>(m, "ImageAlignmentError")
        .def(py::init<>())
        .def_readwrite("image_name", &ImageAlignmentError::image_name)
        .def_readwrite("rotation_error_deg", &ImageAlignmentError::rotation_error_deg)
        .def_readwrite("proj_center_error", &ImageAlignmentError::proj_center_error);

    m.def(
        "align_reconstructions_with_poses",
        py::overload_cast<const Reconstruction&, const Reconstruction&,
                          const double, const double>(&AlignReconstructionsWithPoses),
        py::arg("src"), py::arg("tgt"), py::arg("min_inlier_observations") = 0.3,
        py::arg("max_reproj_error") = 8.0);

    m.def(
        "align_reconstructions_with_poses",
        py::overload_cast<const Reconstruction&, const Reconstruction&,
                          const double>(&AlignReconstructionsWithPoses),
        py::arg("src"), py::arg("tgt"), py::arg("max_proj_center_error"));

    m.def(
        "align_reconstructions_with_points",
        &AlignReconstructionsWithPoints, py::arg("src"), py::arg("tgt"),
        py::arg("min_common_observations") = 3, py::arg("max_error") = 0.005,
        py::arg("min_inlier_ratio") = 0.9);

    m.def(
        "align_reconstrution_to_locations", &AlignReconstructionToLocationsWrapper,
        py::arg("src"), py::arg("image_names"), py::arg("locations"),
        py::arg("min_common_points"), py::arg("ransac_options"));

    m.def(
        "compare_reconstructions",
        [](const Reconstruction& reconstruction1,
           const Reconstruction& reconstruction2,
           const std::string& alignment_error,
           double min_inlier_observations,
           double max_reproj_error,
           double max_proj_center_error) {
            std::vector<ImageAlignmentError> errors;
            Sim3d rec2_from_rec1;
            THROW_CUSTOM_CHECK_MSG(
                CompareModels(reconstruction1,
                              reconstruction2,
                              alignment_error,
                              min_inlier_observations,
                              max_reproj_error,
                              max_proj_center_error,
                              errors,
                              rec2_from_rec1),
                std::runtime_error, "=> Reconstruction alignment failed.");
            return py::dict("rec2_from_rec1"_a = rec2_from_rec1,
                            "errors"_a = errors);
        }, "reconstruction1"_a, "reconstruction2"_a, "alignment_error"_a = "reprojection",
       "min_inlier_observations"_a = 0.3, "max_reproj_error"_a = 8.0,
       "max_proj_center_error"_a = 0.1);
}
