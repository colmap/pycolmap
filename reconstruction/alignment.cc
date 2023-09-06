// Author: Philipp Lindenberger (Phil26AT)

#include "colmap/camera/models.h"
#include "colmap/geometry/projection.h"
#include "colmap/geometry/sim3.h"
#include "colmap/scene/alignment.h"
#include "colmap/scene/reconstruction.h"
#include "colmap/estimators/similarity_transform.h"
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

// TODO(skydes): expose in colmap/exe/model.h
void PrintErrorStats(std::ostream& out, std::vector<double>& vals) {
  const size_t len = vals.size();
  if (len == 0) {
    out << "Cannot extract error statistics from empty input" << std::endl;
    return;
  }
  std::sort(vals.begin(), vals.end());
  out << "Min:    " << vals.front() << std::endl;
  out << "Max:    " << vals.back() << std::endl;
  out << "Mean:   " << Mean(vals) << std::endl;
  out << "Median: " << Median(vals) << std::endl;
  out << "P90:    " << vals[size_t(0.9 * len)] << std::endl;
  out << "P99:    " << vals[size_t(0.99 * len)] << std::endl;
}

void PrintComparisonSummary(std::ostream& out,
                            const std::vector<ImageAlignmentError>& errors) {
  std::vector<double> rotation_errors_deg;
  rotation_errors_deg.reserve(errors.size());
  std::vector<double> proj_center_errors;
  proj_center_errors.reserve(errors.size());
  for (const auto& error : errors) {
    rotation_errors_deg.push_back(error.rotation_error_deg);
    proj_center_errors.push_back(error.proj_center_error);
  }
  out << std::endl << "Rotation errors (degrees)" << std::endl;
  PrintErrorStats(out, rotation_errors_deg);
  out << std::endl << "Projection center errors" << std::endl;
  PrintErrorStats(out, proj_center_errors);
}

py::dict CompareReconstructions(
        const Reconstruction& tgt_reconstruction, const Reconstruction& src_reconstruction,
        const double min_inlier_observations, const double max_reproj_error, bool verbose) {
    THROW_CHECK_GE(min_inlier_observations, 0.0);
    THROW_CHECK_LE(min_inlier_observations, 1.0);

    std::stringstream ss;
    ss << std::endl << "Reconstruction 1" << std::endl;
    ss << StringPrintf("Images: %d", tgt_reconstruction.NumRegImages()) << std::endl;
    ss << StringPrintf("Points: %d", tgt_reconstruction.NumPoints3D()) << std::endl;

    ss << std::endl << "Reconstruction 2" << std::endl;
    ss << StringPrintf("Images: %d", src_reconstruction.NumRegImages()) << std::endl;
    ss << StringPrintf("Points: %d", src_reconstruction.NumPoints3D()) << std::endl;
    if (verbose) {
        py::print(ss.str());
        ss.str("");
    };

    ss << std::endl << "Comparing reconstructed image poses" << std::endl;
    const auto common_image_ids = tgt_reconstruction.FindCommonRegImageIds(src_reconstruction);
    ss << StringPrintf("Common images: %d", common_image_ids.size()) << std::endl;
    if (verbose) {
        py::print(ss.str());
        ss.str("");
    };

    Sim3d tgt_from_src;
    if (!AlignReconstructions(src_reconstruction, tgt_reconstruction,
                              min_inlier_observations, max_reproj_error, &tgt_from_src)) {
        THROW_EXCEPTION(std::runtime_error, "=> Reconstruction alignment failed.");
    }

    ss << "Computed alignment transform:" << std::endl << tgt_from_src.ToMatrix() << std::endl;
    if (verbose) {
        py::print(ss.str());
        ss.str("");
    };

    auto errors = ComputeImageAlignmentError(src_reconstruction, tgt_reconstruction, tgt_from_src);
    std::vector<double> rotation_errors_deg;
    rotation_errors_deg.reserve(errors.size());
    std::vector<double> proj_center_errors;
    proj_center_errors.reserve(errors.size());
    for (const auto& error : errors) {
        rotation_errors_deg.push_back(error.rotation_error_deg);
        proj_center_errors.push_back(error.proj_center_error);
    }
    PrintComparisonSummary(ss, errors);
    if (verbose) {
        py::print(ss.str());
        ss.str("");
    };
    py::dict res("tgt_from_src"_a = tgt_from_src,
                 "rotation_errors"_a = rotation_errors_deg,
                 "proj_center_errors"_a = proj_center_errors);
    return res;
}


template <typename... Args>
using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;

void init_reconstruction_utils(py::module& m) {
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
        "compare_reconstructions", CompareReconstructions,
        py::arg("tgt_reconstruction"), py::arg("src_reconstruction"),
        py::arg("min_inlier_observations") = 0.3, py::arg("max_reproj_error") = 8.0,
        py::arg("verbose") = true);
}
