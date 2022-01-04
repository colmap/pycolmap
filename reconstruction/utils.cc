// Author: Philipp Lindenberger (Phil26AT)

#include "colmap/base/camera_models.h"
#include "colmap/base/projection.h"
#include "colmap/base/reconstruction.h"
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
                                                const std::vector<Eigen::Vector3d>& dst,
                                                double max_error, double min_inlier_ratio,
                                                Eigen::Matrix3x4d* alignment) {
    RANSACOptions ransac_options;
    ransac_options.max_error = max_error;
    ransac_options.min_inlier_ratio = min_inlier_ratio;

    LORANSAC<SimilarityTransformEstimator<3, kEstimateScale>,
             SimilarityTransformEstimator<3, kEstimateScale>>
        ransac(ransac_options);

    const auto report = ransac.Estimate(src, dst);

    if (report.success) {
        *alignment = report.model;
    }
    return report.success;
}

inline SimilarityTransform3 AlignPosesBetweenReconstructions(Reconstruction& self,
                                                             const Reconstruction& ref,
                                                             const double min_inlier_observations,
                                                             double max_reproj_error) {
    THROW_CHECK_GE(min_inlier_observations, 0.0);
    THROW_CHECK_LE(min_inlier_observations, 1.0);
    Eigen::Matrix3x4d alignment;
    bool success = ComputeAlignmentBetweenReconstructions(self, ref, min_inlier_observations,
                                                          max_reproj_error, &alignment);
    THROW_CHECK(success);
    SimilarityTransform3 tform(alignment);
    self.Transform(tform);
    return tform;
}

inline SimilarityTransform3 AlignPointsBetweenReconstructions(Reconstruction& self,
                                                              const Reconstruction& ref,
                                                              int min_overlap, double max_error,
                                                              double min_inlier_ratio) {
    std::vector<Eigen::Vector3d> src;
    std::vector<Eigen::Vector3d> dst;
    // Associate 3D points using point2D_idx
    for (auto& p3D_p : self.Points3D()) {
        // Count how often a 3D point in ref is associated to this 3D point
        std::map<point3D_t, size_t> counts;
        const Track& track = p3D_p.second.Track();
        for (auto& track_el : track.Elements()) {
            if (!ref.IsImageRegistered(track_el.image_id)) {
                continue;
            }
            const Point2D& p2D_dst = ref.Image(track_el.image_id).Point2D(track_el.point2D_idx);
            if (p2D_dst.HasPoint3D()) {
                if (counts.find(p2D_dst.Point3DId()) != counts.end()) {
                    counts[p2D_dst.Point3DId()]++;
                } else {
                    counts[p2D_dst.Point3DId()] = 0;
                }
            }
        }
        if (counts.size() == 0) {
            continue;
        }
        // The 3D point in ref who is associated the most is selected
        auto best_p3D = std::max_element(
            counts.begin(), counts.end(),
            [](const std::pair<point3D_t, size_t>& p1, const std::pair<point3D_t, size_t>& p2) {
                return p1.second < p2.second;
            });
        if (best_p3D->second >= min_overlap) {
            src.push_back(p3D_p.second.XYZ());
            dst.push_back(ref.Point3D(best_p3D->first).XYZ());
        }
    }
    THROW_CHECK_EQ(src.size(), dst.size());
    std::cerr << "Found " << src.size() << " / " << self.NumPoints3D() << " valid correspondences."
              << std::endl;

    Eigen::Matrix3x4d alignment;
    bool success = ComputeRobustAlignmentBetweenPoints<true>(src, dst, max_error, min_inlier_ratio,
                                                             &alignment);
    THROW_CHECK(success);
    SimilarityTransform3 tform(alignment);
    self.Transform(tform);
    return tform;
}

inline SimilarityTransform3 AlignReconstruction(Reconstruction& self,
                                                const std::vector<std::string>& image_names,
                                                const std::vector<Eigen::Vector3d>& locations,
                                                const int min_common_images) {
    SimilarityTransform3 tform;
    THROW_CHECK_GE(min_common_images, 3);
    THROW_CHECK_EQ(image_names.size(), locations.size());
    bool success = self.Align(image_names, locations, min_common_images, &tform);
    THROW_CHECK(success);
    return tform;
}

inline SimilarityTransform3 RobustAlignReconstruction(Reconstruction& self,
                                                      const std::vector<std::string>& image_names,
                                                      const std::vector<Eigen::Vector3d>& locations,
                                                      const int min_common_images, double max_error,
                                                      double min_inlier_ratio) {
    SimilarityTransform3 tform;
    THROW_CHECK_GE(min_common_images, 3);
    THROW_CHECK_EQ(image_names.size(), locations.size());
    RANSACOptions options;
    options.max_error = max_error;
    options.min_inlier_ratio = min_inlier_ratio;
    bool success = self.AlignRobust(image_names, locations, min_common_images, options, &tform);
    THROW_CHECK(success);
    return tform;
}

template <typename... Args>
using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;

void init_reconstruction_utils(py::module& m) {
    m.def(
        "compare_reconstructions",
        [](const Reconstruction& reconstruction1, const Reconstruction& reconstruction2,
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

            Eigen::Matrix3x4d alignment;
            if (!ComputeAlignmentBetweenReconstructions(reconstruction2, reconstruction1,
                                                        min_inlier_observations, max_reproj_error,
                                                        &alignment)) {
                THROW_EXCEPTION(std::runtime_error, "=> Reconstruction alignment failed.");
            }

            const SimilarityTransform3 tform(alignment);
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
            py::dict res("alignment"_a = alignment, "rotation_errors"_a = rotation_errors,
                         "translation_errors"_a = translation_errors,
                         "proj_center_errors"_a = proj_center_errors);
            return res;
        },
        py::arg("src_reconstruction").noconvert(), py::arg("ref_reconstruction").noconvert(),
        py::arg("min_inlier_observations") = 0.3, py::arg("max_reproj_error") = 8.0,
        py::arg("verbose") = true, py::keep_alive<1, 2>(), py::keep_alive<1, 3>());
}
