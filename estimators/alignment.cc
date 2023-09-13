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

template <bool kEstimateScale>
inline bool ComputeRobustAlignmentBetweenPoints(const std::vector<Eigen::Vector3d>& src,
                                                const std::vector<Eigen::Vector3d>& tgt,
                                                double max_error, double min_inlier_ratio,
                                                Sim3d& tgt_from_src) {
    RANSACOptions ransac_options;
    ransac_options.max_error = max_error;
    ransac_options.min_inlier_ratio = min_inlier_ratio;

    LORANSAC<SimilarityTransformEstimator<3, kEstimateScale>,
             SimilarityTransformEstimator<3, kEstimateScale>>
        ransac(ransac_options);

    const auto report = ransac.Estimate(src, tgt);

    if (report.success) {
        tgt_from_src.scale = report.model.col(0).norm();
        tgt_from_src.rotation = Eigen::Quaterniond(report.model.template leftCols<3>()
                                                   / tgt_from_src.scale).normalized();
        tgt_from_src.translation = report.model.template rightCols<1>();
    }
    return report.success;
}

inline Sim3d AlignReconstructionsWithPoses(const Reconstruction& src,
                                           const Reconstruction& tgt,
                                           const double min_inlier_observations,
                                           const double max_reproj_error) {
    THROW_CHECK_GE(min_inlier_observations, 0.0);
    THROW_CHECK_LE(min_inlier_observations, 1.0);
    Sim3d tgt_from_src;
    bool success = AlignReconstructions(src, tgt, min_inlier_observations,
                                        max_reproj_error, &tgt_from_src);
    THROW_CHECK(success);
    return tgt_from_src;
}

inline Sim3d AlignReconstructionsWithPoses(const Reconstruction& src,
                                           const Reconstruction& tgt,
                                           const double max_proj_center_error) {
    THROW_CHECK_GT(max_proj_center_error, 0.0);
    Sim3d tgt_from_src;
    bool success = AlignReconstructions(src, tgt, max_proj_center_error, &tgt_from_src);
    THROW_CHECK(success);
    return tgt_from_src;
}

inline Sim3d AlignReconstructionsWithPoints(const Reconstruction& src,
                                            const Reconstruction& tgt,
                                            const int min_overlap,
                                            const double max_error,
                                            const double min_inlier_ratio) {
    std::vector<Eigen::Vector3d> p_src;
    std::vector<Eigen::Vector3d> p_tgt;
    // Associate 3D points using point2D_idx
    for (auto& p3D_p : src.Points3D()) {
        // Count how often a 3D point in tgt is associated to this 3D point
        std::map<point3D_t, size_t> counts;
        const Track& track = p3D_p.second.Track();
        for (auto& track_el : track.Elements()) {
            if (!tgt.IsImageRegistered(track_el.image_id)) {
                continue;
            }
            const Point2D& p2D_tgt = tgt.Image(track_el.image_id).Point2D(track_el.point2D_idx);
            if (p2D_tgt.HasPoint3D()) {
                if (counts.find(p2D_tgt.point3D_id) != counts.end()) {
                    counts[p2D_tgt.point3D_id]++;
                } else {
                    counts[p2D_tgt.point3D_id] = 0;
                }
            }
        }
        if (counts.size() == 0) {
            continue;
        }
        // The 3D point in tgt who is associated the most is selected
        auto best_p3D = std::max_element(
            counts.begin(), counts.end(),
            [](const std::pair<point3D_t, size_t>& p1, const std::pair<point3D_t, size_t>& p2) {
                return p1.second < p2.second;
            });
        if (best_p3D->second >= min_overlap) {
            p_src.push_back(p3D_p.second.XYZ());
            p_tgt.push_back(tgt.Point3D(best_p3D->first).XYZ());
        }
    }
    THROW_CHECK_EQ(p_src.size(), p_tgt.size());
    std::cerr << "Found " << p_src.size() << " / " << src.NumPoints3D() << " valid correspondences."
              << std::endl;

    Sim3d tgt_from_src;
    bool success = ComputeRobustAlignmentBetweenPoints<true>(p_src, p_tgt, max_error,
                                                             min_inlier_ratio, tgt_from_src);
    THROW_CHECK(success);
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
    bool success = AlignReconstructionToLocations(src, image_names, locations,
                                                  min_common_images, ransac_options,
                                                  &locationsFromSrc);
    THROW_CHECK(success);
    return locationsFromSrc;
}

template <typename... Args>
using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;

void bind_alignment(py::module& m) {
    py::class_<ImageAlignmentError>(m, "ImageAlignmentError")
        .def(py::init<>())
        .def_readwrite("image_name", &ImageAlignmentError::image_name)
        .def_readwrite("rotation_error_deg", &ImageAlignmentError::rotation_error_deg)
        .def_readwrite("proj_center_error", &ImageAlignmentError::proj_center_error);

    m.def(
        "align_reconstructions_with_poses",
        static_cast<Sim3d (*)(const Reconstruction&, const Reconstruction&,
                              const double, const double)>(&AlignReconstructionsWithPoses),
        py::arg("src"), py::arg("tgt"), py::arg("min_inlier_observations"),
        py::arg("max_reproj_error"));

    m.def(
        "align_reconstructions_with_poses",
        static_cast<Sim3d (*)(const Reconstruction&, const Reconstruction&,
                              const double)>(&AlignReconstructionsWithPoses),
        py::arg("src"), py::arg("tgt"), py::arg("max_proj_center_error"));

    m.def(
        "align_reconstructions_with_points",
        &AlignReconstructionsWithPoints, py::arg("src"), py::arg("tgt"),
        py::arg("min_overlap"), py::arg("max_error"), py::arg("min_inlier_ratio"));

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
