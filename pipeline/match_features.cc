// Author: Philipp Lindenberger (Phil26AT)

#include "colmap/base/camera_models.h"
#include "colmap/base/image_reader.h"
#include "colmap/base/reconstruction.h"
#include "colmap/controllers/incremental_mapper.h"
#include "colmap/exe/feature.h"
#include "colmap/exe/sfm.h"
#include "colmap/feature/extraction.h"
#include "colmap/feature/matching.h"
#include "colmap/feature/sift.h"
#include "colmap/util/misc.h"

using namespace colmap;

#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;
using namespace pybind11::literals;

#include "helpers.h"
#include "log_exceptions.h"
#include "utils.h"

template <typename Matcher, typename Opts>
void match_features(py::object database_path_,
                    SiftMatchingOptions sift_options,
                    Opts matching_options,
                    const Device device,
                    bool verbose) {
    std::string database_path = py::str(database_path_).cast<std::string>();
    THROW_CHECK_FILE_EXISTS(database_path);
    try {
        py::cast(matching_options).attr("check").attr("__call__")();
    } catch (py::error_already_set& ex) {
        // Allow pass if no check function defined.
        if (!ex.matches(PyExc_AttributeError)) {
            throw ex;
        }
    }

    sift_options.use_gpu = IsGPU(device);
    VerifySiftGPUParams(sift_options.use_gpu);
    py::gil_scoped_release release;
    Matcher feature_matcher(matching_options, sift_options, database_path);

    std::stringstream oss;
    std::streambuf* oldcerr = nullptr;
    std::streambuf* oldcout = nullptr;
    if (!verbose) {
        oldcout = std::cout.rdbuf(oss.rdbuf());
    }

    feature_matcher.Start();
    PyWait(&feature_matcher);

    if (!verbose) {
        std::cout.rdbuf(oldcout);
    }
}

void match_exhaustive(py::object database_path_,
                      SiftMatchingOptions sift_options,
                      int block_size,
                      const Device device,
                      bool verbose) {
    ExhaustiveMatchingOptions options;
    options.block_size = block_size;
    match_features<ExhaustiveFeatureMatcher>(
        database_path_, sift_options, options, device, verbose);
}

void verify_matches(const py::object database_path_,
                    const py::object pairs_path_,
                    const int max_num_trials,
                    const float min_inlier_ratio) {
    std::string database_path = py::str(database_path_).cast<std::string>();
    THROW_CHECK_FILE_EXISTS(database_path);
    std::string pairs_path = py::str(pairs_path_).cast<std::string>();
    THROW_CHECK_FILE_EXISTS(pairs_path);
    py::gil_scoped_release release;  // verification is multi-threaded

    SiftMatchingOptions options;
    options.use_gpu = false;
    options.max_num_trials = max_num_trials;
    options.min_inlier_ratio = min_inlier_ratio;

    ImagePairsMatchingOptions matcher_options;
    matcher_options.match_list_path = pairs_path;

    ImagePairsFeatureMatcher feature_matcher(
        matcher_options, options, database_path);
    feature_matcher.Start();
    PyWait(&feature_matcher);
}

void init_match_features(py::module& m) {
    /* OPTIONS */
    using SMOpts = SiftMatchingOptions;
    auto PySiftMatchingOptions =
        py::class_<SMOpts>(m, "SiftMatchingOptions")
            .def(py::init<>())
            .def_readwrite("num_threads", &SMOpts::num_threads)
            .def_readwrite("gpu_index",
                           &SMOpts::gpu_index,
                           "Index of the GPU used for feature matching. For "
                           "multi-GPU matching, "
                           "you should separate multiple GPU indices by comma, "
                           "e.g., \"0,1,2,3\".")
            .def_readwrite(
                "max_ratio",
                &SMOpts::max_ratio,
                "Maximum distance ratio between first and second best match.")
            .def_readwrite("max_distance",
                           &SMOpts::max_distance,
                           "Maximum distance to best match.")
            .def_readwrite("cross_check",
                           &SMOpts::cross_check,
                           "Whether to enable cross checking in matching.")
            .def_readwrite("max_num_matches",
                           &SMOpts::max_num_matches,
                           "Maximum number of matches.")
            .def_readwrite(
                "max_error",
                &SMOpts::max_error,
                "Maximum epipolar error in pixels for geometric verification.")
            .def_readwrite("confidence",
                           &SMOpts::confidence,
                           "Confidence threshold for geometric verification.")
            .def_readwrite(
                "min_num_trials",
                &SMOpts::min_num_trials,
                "Minimum number of RANSAC iterations. Note that this option "
                "overrules the min_inlier_ratio option.")
            .def_readwrite(
                "max_num_trials",
                &SMOpts::max_num_trials,
                "Maximum number of RANSAC iterations. Note that this option "
                "overrules the min_inlier_ratio option.")
            .def_readwrite("min_inlier_ratio",
                           &SMOpts::min_inlier_ratio,
                           "A priori assumed minimum inlier ratio, which "
                           "determines the maximum "
                           "number of iterations.")
            .def_readwrite("min_num_inliers",
                           &SMOpts::min_num_inliers,
                           "Minimum number of inliers for an image pair to be "
                           "considered as "
                           "geometrically verified.")
            .def_readwrite("multiple_models",
                           &SMOpts::multiple_models,
                           "Whether to attempt to estimate multiple geometric "
                           "models per image pair.")
            .def_readwrite("guided_matching",
                           &SMOpts::guided_matching,
                           "Whether to perform guided matching, if geometric "
                           "verification succeeds.")
            .def_readwrite("planar_scene",
                           &SMOpts::planar_scene,
                           "Force Homography use for Two-view Geometry (can "
                           "help for planar scenes)");
    make_dataclass(PySiftMatchingOptions);
    auto sift_matching_options = PySiftMatchingOptions().cast<SMOpts>();

    using EMOpts = ExhaustiveMatchingOptions;
    auto PyExhaustiveMatchingOptions =
        py::class_<ExhaustiveMatchingOptions>(m, "ExhaustiveMatchingOptions")
            .def(py::init<>())
            .def_readwrite("block_size", &EMOpts::block_size);
    make_dataclass(PyExhaustiveMatchingOptions);
    auto exhaustive_options = PyExhaustiveMatchingOptions().cast<EMOpts>();

    using SeqMOpts = SequentialMatchingOptions;
    auto PySequentialMatchingOptions =
        py::class_<SeqMOpts>(m, "SequentialMatchingOptions")
            .def(py::init<>())
            .def_readwrite("overlap",
                           &SeqMOpts::overlap,
                           "Number of overlapping image pairs.")
            .def_readwrite(
                "quadratic_overlap",
                &SeqMOpts::quadratic_overlap,
                "Whether to match images against their quadratic neighbors.")
            .def_readwrite("loop_detection",
                           &SeqMOpts::loop_detection,
                           "Loop detection is invoked every "
                           "`loop_detection_period` images.")
            .def_readwrite("loop_detection_num_images",
                           &SeqMOpts::loop_detection_num_images,
                           "The number of images to retrieve in loop "
                           "detection. This number should be significantly "
                           "bigger than the sequential matching overlap.")
            .def_readwrite(
                "loop_detection_num_nearest_neighbors",
                &SeqMOpts::loop_detection_num_nearest_neighbors,
                "Number of nearest neighbors to retrieve per query feature.")
            .def_readwrite(
                "loop_detection_num_checks",
                &SeqMOpts::loop_detection_num_checks,
                "Number of nearest-neighbor checks to use in retrieval.")
            .def_readwrite(
                "loop_detection_num_images_after_verification",
                &SeqMOpts::loop_detection_num_images_after_verification,
                "How many images to return after spatial verification. Set to "
                "0 to turn off spatial verification.")
            .def_readwrite("loop_detection_max_num_features",
                           &SeqMOpts::loop_detection_max_num_features,
                           "The maximum number of features to use for indexing "
                           "an image. If an image has more features, only the "
                           "largest-scale features will be indexed.")
            .def_readwrite("vocab_tree_path",
                           &SeqMOpts::vocab_tree_path,
                           "Path to the vocabulary tree.");
    make_dataclass(PySequentialMatchingOptions);
    auto sequential_options = PySequentialMatchingOptions().cast<SeqMOpts>();

    using SpMOpts = SpatialMatchingOptions;
    auto PySpatialMatchingOptions =
        py::class_<SpMOpts>(m, "SpatialMatchingOptions")
            .def(py::init<>())
            .def_readwrite("is_gps",
                           &SpMOpts::is_gps,
                           "Whether the location priors in the database are "
                           "GPS coordinates in the form of longitude and "
                           "latitude coordinates in degrees.")
            .def_readwrite(
                "ignore_z",
                &SpMOpts::ignore_z,
                "Whether to ignore the Z-component of the location prior.")
            .def_readwrite("max_num_neighbors",
                           &SpMOpts::max_num_neighbors,
                           "The maximum number of nearest neighbors to match.")
            .def_readwrite("max_distance",
                           &SpMOpts::max_distance,
                           "The maximum distance between the query and nearest "
                           "neighbor [meters].");
    make_dataclass(PySpatialMatchingOptions);
    auto spatial_options = PySpatialMatchingOptions().cast<SpMOpts>();

    using VTMOpts = VocabTreeMatchingOptions;
    auto PyVocabTreeMatchingOptions =
        py::class_<VTMOpts>(m, "VocabTreeMatchingOptions")
            .def(py::init<>())
            .def_readwrite("num_images",
                           &VTMOpts::num_images,
                           "Number of images to retrieve for each query image.")
            .def_readwrite(
                "num_nearest_neighbors",
                &VTMOpts::num_nearest_neighbors,
                "Number of nearest neighbors to retrieve per query feature.")
            .def_readwrite(
                "num_checks",
                &VTMOpts::num_checks,
                "Number of nearest-neighbor checks to use in retrieval.")
            .def_readwrite(
                "num_images_after_verification",
                &VTMOpts::num_images_after_verification,
                "How many images to return after spatial verification. Set to "
                "0 to turn off spatial verification.")
            .def_readwrite(
                "max_num_features",
                &VTMOpts::max_num_features,
                "The maximum number of features to use for indexing an image.")
            .def_readwrite("vocab_tree_path",
                           &VTMOpts::vocab_tree_path,
                           "Path to the vocabulary tree.")
            .def_readwrite(
                "match_list_path",
                &VTMOpts::match_list_path,
                "Optional path to file with specific image names to match.")
            .def("check", [](VTMOpts& self) {
                THROW_CHECK_MSG(!self.vocab_tree_path.empty(),
                                "vocab_tree_path required.");
                THROW_CHECK_FILE_EXISTS(self.vocab_tree_path);
            });
    make_dataclass(PyVocabTreeMatchingOptions);
    auto vocabtree_options = PyVocabTreeMatchingOptions().cast<VTMOpts>();

    m.def("match_exhaustive",
          &match_exhaustive,
          py::arg("database_path"),
          py::arg("sift_options") = sift_matching_options,
          py::arg("block_size") = 50,
          py::arg("device") = Device::AUTO,
          py::arg("verbose") = true,
          "Exhaustive feature matching");

    m.def("match_exhaustive",
          &match_features<ExhaustiveFeatureMatcher, EMOpts>,
          py::arg("database_path"),
          py::arg("sift_options") = sift_matching_options,
          py::arg("matching_options") = exhaustive_options,
          py::arg("device") = Device::AUTO,
          py::arg("verbose") = true,
          "Sequential feature matching");

    m.def("match_sequential",
          &match_features<SequentialFeatureMatcher, SeqMOpts>,
          py::arg("database_path"),
          py::arg("sift_options") = sift_matching_options,
          py::arg("matching_options") = sequential_options,
          py::arg("device") = Device::AUTO,
          py::arg("verbose") = true,
          "Sequential feature matching");

    m.def("match_spatial",
          &match_features<SpatialFeatureMatcher, SpMOpts>,
          py::arg("database_path"),
          py::arg("sift_options") = sift_matching_options,
          py::arg("matching_options") = spatial_options,
          py::arg("device") = Device::AUTO,
          py::arg("verbose") = true,
          "Spatial feature matching");

    m.def("match_vocabtree",
          &match_features<VocabTreeFeatureMatcher, VTMOpts>,
          py::arg("database_path"),
          py::arg("sift_options") = sift_matching_options,
          py::arg("matching_options") = vocabtree_options,
          py::arg("device") = Device::AUTO,
          py::arg("verbose") = true,
          "Vocab tree feature matching");

    m.def("verify_matches",
          &verify_matches,
          py::arg("database_path"),
          py::arg("pairs_path"),
          py::arg("max_num_trials") = SiftMatchingOptions().max_num_trials,
          py::arg("min_inlier_ratio") = SiftMatchingOptions().min_inlier_ratio,
          "Run geometric verification of the matches");
}
