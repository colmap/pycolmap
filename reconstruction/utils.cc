// Author: Philipp Lindenberger (Phil26AT)

#include "colmap/camera/models.h"
#include "colmap/geometry/projection.h"
#include "colmap/base/alignment.h"
#include "colmap/base/reconstruction.h"
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
                                                SimilarityTransform3* tgtFromSrc) {
    RANSACOptions ransac_options;
    ransac_options.max_error = max_error;
    ransac_options.min_inlier_ratio = min_inlier_ratio;

    LORANSAC<SimilarityTransformEstimator<3, kEstimateScale>,
             SimilarityTransformEstimator<3, kEstimateScale>>
        ransac(ransac_options);

    const auto report = ransac.Estimate(src, tgt);

    if (report.success) {
        *tgtFromSrc = SimilarityTransform3(report.model);
    }
    return report.success;
}

inline SimilarityTransform3 AlignReconstructionsWithPoses(const Reconstruction& src,
                                                          const Reconstruction& tgt,
                                                          const double min_inlier_observations,
                                                          const double max_reproj_error) {
    THROW_CHECK_GE(min_inlier_observations, 0.0);
    THROW_CHECK_LE(min_inlier_observations, 1.0);
    SimilarityTransform3 tgtFromSrc;
    bool success = AlignReconstructions(src, tgt, min_inlier_observations,
                                        max_reproj_error, &tgtFromSrc);
    THROW_CHECK(success);
    return tgtFromSrc;
}

inline SimilarityTransform3 AlignReconstructionsWithPoses(const Reconstruction& src,
                                                          const Reconstruction& tgt,
                                                          const double max_proj_center_error) {
    THROW_CHECK_GT(max_proj_center_error, 0.0);
    SimilarityTransform3 tgtFromSrc;
    bool success = AlignReconstructions(src, tgt, max_proj_center_error, &tgtFromSrc);
    THROW_CHECK(success);
    return tgtFromSrc;
}

inline SimilarityTransform3 AlignReconstructionsWithPoints(const Reconstruction& src,
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
                if (counts.find(p2D_tgt.Point3DId()) != counts.end()) {
                    counts[p2D_tgt.Point3DId()]++;
                } else {
                    counts[p2D_tgt.Point3DId()] = 0;
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

    SimilarityTransform3 tgtFromSrc;
    bool success = ComputeRobustAlignmentBetweenPoints<true>(p_src, p_tgt, max_error,
                                                             min_inlier_ratio, &tgtFromSrc);
    THROW_CHECK(success);
    return tgtFromSrc;
}

inline SimilarityTransform3 AlignReconstructionToLocationsWrapper(
        const Reconstruction& src,
        const std::vector<std::string>& image_names,
        const std::vector<Eigen::Vector3d>& locations,
        const int min_common_images,
        const RANSACOptions& ransac_options) {
    THROW_CHECK_GE(min_common_images, 3);
    THROW_CHECK_EQ(image_names.size(), locations.size());
    SimilarityTransform3 locationsFromSrc;
    bool success = AlignReconstructionToLocations(src, image_names, locations,
                                                  min_common_images, ransac_options,
                                                  &locationsFromSrc);
    THROW_CHECK(success);
    return locationsFromSrc;
}

py::dict CompareReconstructions(
        const Reconstruction& reconstruction1, const Reconstruction& reconstruction2,
        const double min_inlier_observations, const double max_reproj_error, bool verbose) {
    THROW_CHECK_GE(min_inlier_observations, 0.0);
    THROW_CHECK_LE(min_inlier_observations, 1.0);
    auto PrintComparisonSummary = [](std::stringstream& ss,
                                     std::vector<double>& rotation_errors,
                                     std::vector<double>& translation_errors,
                                     std::vector<double>& proj_center_errors) {
        auto PrintErrorStats = [](std::stringstream& ss, std::vector<double>& vals) {
            const size_t len = vals.size();
            if (len == 0) {
                ss << "Cannot extract error statistics from empty input" << std::endl;
                return;
            }
            std::sort(vals.begin(), vals.end());
            ss << "Min:    " << vals.front() << std::endl;
            ss << "Max:    " << vals.back() << std::endl;
            ss << "Mean:   " << Mean(vals) << std::endl;
            ss << "Median: " << Median(vals) << std::endl;
            ss << "P90:    " << vals[size_t(0.9 * len)] << std::endl;
            ss << "P99:    " << vals[size_t(0.99 * len)] << std::endl;
        };
        ss << "# Image pose error summary" << std::endl;
        ss << std::endl << "Rotation angular errors (degrees)" << std::endl;
        PrintErrorStats(ss, rotation_errors);
        ss << std::endl << "Translation distance errors" << std::endl;
        PrintErrorStats(ss, translation_errors);
        ss << std::endl << "Projection center distance errors" << std::endl;
        PrintErrorStats(ss, proj_center_errors);
    };

    std::stringstream ss;
    ss << std::endl << "Reconstruction 1" << std::endl;
    ss << StringPrintf("Images: %d", reconstruction1.NumRegImages()) << std::endl;
    ss << StringPrintf("Points: %d", reconstruction1.NumPoints3D()) << std::endl;

    ss << std::endl << "Reconstruction 2" << std::endl;
    ss << StringPrintf("Images: %d", reconstruction2.NumRegImages()) << std::endl;
    ss << StringPrintf("Points: %d", reconstruction2.NumPoints3D()) << std::endl;
    if (verbose) {
        py::print(ss.str());
        ss.str("");
    };

    ss << std::endl << "Comparing reconstructed image poses" << std::endl;
    const auto common_image_ids = reconstruction1.FindCommonRegImageIds(reconstruction2);
    ss << StringPrintf("Common images: %d", common_image_ids.size()) << std::endl;
    if (verbose) {
        py::print(ss.str());
        ss.str("");
    };

    SimilarityTransform3 tform;
    if (!AlignReconstructions(reconstruction2, reconstruction1,
                              min_inlier_observations, max_reproj_error, &tform)) {
        THROW_EXCEPTION(std::runtime_error, "=> Reconstruction alignment failed.");
    }

    ss << "Computed alignment transform:" << std::endl << tform.Matrix() << std::endl;
    if (verbose) {
        py::print(ss.str());
        ss.str("");
    };

    const size_t num_images = common_image_ids.size();
    std::vector<double> rotation_errors(num_images, 0.0);
    std::vector<double> translation_errors(num_images, 0.0);
    std::vector<double> proj_center_errors(num_images, 0.0);
    for (size_t i = 0; i < num_images; ++i) {
        const image_t image_id = common_image_ids[i];
        Image image1 = reconstruction1.Image(image_id);  // copy!
        Image image2 = reconstruction2.Image(image_id);  // copy!
        tform.TransformPose(&image2.Qvec(), &image2.Tvec());

        const Eigen::Vector4d normalized_qvec1 = NormalizeQuaternion(image1.Qvec());
        const Eigen::Quaterniond quat1(normalized_qvec1(0), normalized_qvec1(1),
                                       normalized_qvec1(2), normalized_qvec1(3));
        const Eigen::Vector4d normalized_qvec2 = NormalizeQuaternion(image2.Qvec());
        const Eigen::Quaterniond quat2(normalized_qvec2(0), normalized_qvec2(1),
                                       normalized_qvec2(2), normalized_qvec2(3));

        rotation_errors[i] = RadToDeg(quat1.angularDistance(quat2));
        translation_errors[i] = (image1.Tvec() - image2.Tvec()).norm();
        proj_center_errors[i] =
            (image1.ProjectionCenter() - image2.ProjectionCenter()).norm();
    }
    PrintComparisonSummary(ss, rotation_errors, translation_errors, proj_center_errors);
    if (verbose) {
        py::print(ss.str());
        ss.str("");
    };
    py::dict res("alignment"_a = tform, "rotation_errors"_a = rotation_errors,
                 "translation_errors"_a = translation_errors,
                 "proj_center_errors"_a = proj_center_errors);
    return res;
}


template <typename... Args>
using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;

void init_reconstruction_utils(py::module& m) {
    m.def(
        "align_reconstructions_with_poses",
        static_cast<SimilarityTransform3 (*)(const Reconstruction&, const Reconstruction&,
                                             const double, const double)>(&AlignReconstructionsWithPoses),
        py::arg("src"), py::arg("tgt"), py::arg("min_inlier_observations"),
        py::arg("max_reproj_error"));

    m.def(
        "align_reconstructions_with_poses",
        static_cast<SimilarityTransform3 (*)(const Reconstruction&, const Reconstruction&,
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
        py::arg("src_reconstruction"), py::arg("tgt_reconstruction"),
        py::arg("min_inlier_observations") = 0.3, py::arg("max_reproj_error") = 8.0,
        py::arg("verbose") = true);
}
