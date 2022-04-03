// Author: Paul-Edouard Sarlin (skydes)

#include "colmap/base/reconstruction.h"
#include "colmap/base/image_reader.h"
#include "colmap/base/camera_models.h"
#include "colmap/util/misc.h"
#include "colmap/feature/extraction.h"
#include "colmap/feature/sift.h"
#include "colmap/feature/matching.h"
#include "colmap/controllers/incremental_mapper.h"
#include "colmap/exe/feature.h"
#include "colmap/exe/sfm.h"

using namespace colmap;

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/iostream.h>

namespace py = pybind11;
using namespace pybind11::literals;

#include "log_exceptions.h"
#include "helpers.h"


void VerifyCameraParams(const std::string& camera_model,
                        const std::string& params) {
  THROW_CUSTOM_CHECK_MSG(
      ExistsCameraModelWithName(camera_model),
      std::invalid_argument,
      (std::string("Invalid camera model: ")+ camera_model).c_str()
  );

  const std::vector<double> camera_params = CSVToVector<double>(params);
  const int camera_model_id = CameraModelNameToId(camera_model);

  if (camera_params.size() > 0 &&
      !CameraModelVerifyParams(camera_model_id, camera_params)) {
    THROW_EXCEPTION(std::invalid_argument,
                    "Invalid camera parameters.");
  }
}

void VerifySiftGPUParams(const bool use_gpu) {
#ifndef CUDA_ENABLED
  if (use_gpu) {
    THROW_EXCEPTION(std::invalid_argument,
                    "Cannot use Sift GPU without CUDA support; "
                    "set device='auto' or device='cpu'.")
  }
#endif
}


void extract_features(const py::object database_path_,
                      const py::object image_path_,
                      const std::vector<std::string> image_list,
                      const CameraMode camera_mode,
                      const std::string camera_model,
                      ImageReaderOptions reader_options,
                      SiftExtractionOptions sift_options,
                      const Device device,
                      bool verbose) {
  std::string database_path = py::str(database_path_).cast<std::string>();
  THROW_CHECK_MSG(!ExistsFile(database_path), database_path + " already exists.");
  THROW_CHECK_HAS_FILE_EXTENSION(database_path, ".db");
  THROW_CHECK_FILE_OPEN(database_path);
  std::string image_path = py::str(image_path_).cast<std::string>();
  THROW_CHECK_DIR_EXISTS(image_path);
  sift_options.use_gpu = IsGPU(device);
  VerifySiftGPUParams(sift_options.use_gpu);

  UpdateImageReaderOptionsFromCameraMode(reader_options, camera_mode);
  reader_options.camera_model = camera_model;

  reader_options.database_path = database_path;
  reader_options.image_path = image_path;

  if (!image_list.empty()) {
    reader_options.image_list = image_list;
  }

  THROW_CHECK(ExistsCameraModelWithName(reader_options.camera_model));

  VerifyCameraParams(reader_options.camera_model,
                     reader_options.camera_params);

  std::stringstream oss;
  std::streambuf* oldcerr = nullptr;
  std::streambuf* oldcout = nullptr;
  if (!verbose) {
    oldcout = std::cout.rdbuf( oss.rdbuf() );
  }
  py::gil_scoped_release release;
  SiftFeatureExtractor feature_extractor(reader_options,
                                         sift_options);

  feature_extractor.Start();
  feature_extractor.Wait();

  if (!verbose) {
    std::cout.rdbuf(oldcout);
  }
}

template<typename Matcher, typename Opts>
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
    if (!ex.matches(PyExc_AttributeError) &&
        !ex.matches(PyExc_SystemError)) {
      throw;
    }
  }

  sift_options.use_gpu = IsGPU(device);
  VerifySiftGPUParams(sift_options.use_gpu);
  py::gil_scoped_release release;
  Matcher feature_matcher(matching_options,
                          sift_options,
                          database_path);

  std::stringstream oss;
  std::streambuf* oldcerr = nullptr;
  std::streambuf* oldcout = nullptr;
  if (!verbose) {
    oldcout = std::cout.rdbuf( oss.rdbuf() );
  }

  feature_matcher.Start();
  feature_matcher.Wait();

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
    database_path_, sift_options, options, device, verbose
  );
}


void import_images(
        const py::object database_path_,
        const py::object image_path_,
        const CameraMode camera_mode,
        const std::string camera_model,
        const std::vector<std::string> image_list
) {
    std::string database_path = py::str(database_path_).cast<std::string>();
    THROW_CHECK_FILE_EXISTS(database_path);
    std::string image_path = py::str(image_path_).cast<std::string>();
    THROW_CHECK_DIR_EXISTS(image_path);

    ImageReaderOptions options;
    options.database_path = database_path;
    options.image_path = image_path;
    options.image_list = image_list;
    UpdateImageReaderOptionsFromCameraMode(options, camera_mode);
    if (!camera_model.empty()) {
        options.camera_model = camera_model;
    }
    THROW_CUSTOM_CHECK_MSG(
        ExistsCameraModelWithName(options.camera_model),
        std::invalid_argument,
        (std::string("Invalid camera model: ")+ options.camera_model).c_str()
    );


    Database database(options.database_path);
    ImageReader image_reader(options, &database);

    PyInterrupt py_interrupt(2.0);

    while (image_reader.NextIndex() < image_reader.NumImages()) {
        if (py_interrupt.Raised()) {
            throw py::error_already_set();
        }
        Camera camera;
        Image image;
        Bitmap bitmap;
        if (image_reader.Next(&camera, &image, &bitmap, nullptr) !=
            ImageReader::Status::SUCCESS) {
            continue;
        }
        DatabaseTransaction database_transaction(&database);
        if (image.ImageId() == kInvalidImageId) {
            image.SetImageId(database.WriteImage(image));
        }
    }
}


Camera infer_camera_from_image(const py::object image_path_) {
    std::string image_path = py::str(image_path_).cast<std::string>();
    THROW_CHECK_FILE_EXISTS(image_path);

    Bitmap bitmap;
    THROW_CUSTOM_CHECK_MSG(
        bitmap.Read(image_path, false),
        std::invalid_argument,
        (std::string("Cannot read image file: ") + image_path).c_str()
    );

    ImageReaderOptions options;
    Camera camera;
    camera.SetCameraId(kInvalidCameraId);
    camera.SetModelIdFromName(options.camera_model);
    double focal_length = 0.0;
    if (bitmap.ExifFocalLength(&focal_length)) {
        camera.SetPriorFocalLength(true);
    } else {
        focal_length = options.default_focal_length_factor *
                       std::max(bitmap.Width(), bitmap.Height());
        camera.SetPriorFocalLength(false);
    }
    camera.InitializeWithId(camera.ModelId(), focal_length,
                            bitmap.Width(), bitmap.Height());
    THROW_CUSTOM_CHECK_MSG(
        camera.VerifyParams(),
        std::invalid_argument,
        (std::string("Invalid camera params: ") + camera.ParamsToString()).c_str()
    );

    return camera;
}

void verify_matches(
        const py::object database_path_,
        const py::object pairs_path_,
        const int max_num_trials,
        const float min_inlier_ratio
) {
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
    feature_matcher.Wait();
}

Reconstruction triangulate_points(
        Reconstruction reconstruction,
        const py::object database_path_,
        const py::object image_path_,
        const py::object output_path_,
        const bool clear_points,
        const IncrementalMapperOptions& options) {
    std::string database_path = py::str(database_path_).cast<std::string>();
    THROW_CHECK_FILE_EXISTS(database_path);
    std::string image_path = py::str(image_path_).cast<std::string>();
    THROW_CHECK_DIR_EXISTS(image_path);
    std::string output_path = py::str(output_path_).cast<std::string>();
    CreateDirIfNotExists(output_path);

    py::gil_scoped_release release;
    RunPointTriangulatorImpl(
        reconstruction,
        database_path,
        image_path,
        output_path,
        options,
        clear_points);
    return reconstruction;
}

// Copied from colmap/exe/sfm.cc
std::map<size_t, Reconstruction> incremental_mapping(
        const py::object database_path_,
        const py::object image_path_,
        const py::object output_path_,
        const IncrementalMapperOptions& options) {
    std::string database_path = py::str(database_path_).cast<std::string>();
    THROW_CHECK_FILE_EXISTS(database_path);
    std::string image_path = py::str(image_path_).cast<std::string>();
    THROW_CHECK_DIR_EXISTS(image_path);
    std::string output_path = py::str(output_path_).cast<std::string>();
    CreateDirIfNotExists(output_path);

    py::gil_scoped_release release;
    ReconstructionManager reconstruction_manager;
    IncrementalMapperController mapper(
        &options, image_path, database_path, &reconstruction_manager);

    // In case a new reconstruction is started, write results of individual sub-
    // models to as their reconstruction finishes instead of writing all results
    // after all reconstructions finished.
    size_t prev_num_reconstructions = 0;
    std::map<size_t, Reconstruction> reconstructions;
    mapper.AddCallback(
        IncrementalMapperController::LAST_IMAGE_REG_CALLBACK, [&]() {
            // If the number of reconstructions has not changed, the last model
            // was discarded for some reason.
            if (reconstruction_manager.Size() > prev_num_reconstructions) {
                const std::string reconstruction_path = JoinPaths(
                    output_path, std::to_string(prev_num_reconstructions));
                const auto& reconstruction =
                    reconstruction_manager.Get(prev_num_reconstructions);
                CreateDirIfNotExists(reconstruction_path);
                reconstruction.Write(reconstruction_path);
                reconstructions[prev_num_reconstructions] = reconstruction;
                prev_num_reconstructions = reconstruction_manager.Size();
            }
    });

    PyInterrupt py_interrupt(1.0); // Check for interrupts every 2 seconds
    mapper.AddCallback(
        IncrementalMapperController::NEXT_IMAGE_REG_CALLBACK, [&]() {
            if (py_interrupt.Raised()) {
                throw py::error_already_set();
            }
    });

    mapper.Start();
    mapper.Wait();
    return reconstructions;
}

std::map<size_t, Reconstruction> incremental_mapping(
        const py::object database_path_,
        const py::object image_path_,
        const py::object output_path_,
        const int num_threads,
        const int min_num_matches) {
    IncrementalMapperOptions options;
    options.num_threads = num_threads;
    options.min_num_matches = min_num_matches;
    return incremental_mapping(
        database_path_, image_path_, output_path_, options);
}

void init_pipeline(py::module& m) {
    /* OPTIONS */
    auto PyCameraMode = py::enum_<CameraMode>(m, "CameraMode")
        .value("AUTO", CameraMode::AUTO)
        .value("SINGLE", CameraMode::SINGLE)
        .value("PER_FOLDER", CameraMode::PER_FOLDER)
        .value("PER_IMAGE", CameraMode::PER_IMAGE);
    AddStringToEnumConstructor(PyCameraMode);

    using IROpts = ImageReaderOptions;
    auto PyImageReaderOptions =
        py::class_<IROpts>(m, "ImageReaderOptions")
          .def(py::init<>())
          .def_readwrite("existing_camera_id", &IROpts::existing_camera_id,
                         "Whether to explicitly use an existing camera for all images. "
                         "Note that in this case the specified camera model and parameters are ignored.")
          .def_readwrite("camera_params", &IROpts::camera_params,
                         "Manual specification of camera parameters. If empty, camera parameters "
                         "will be extracted from EXIF, i.e. principal point and focal length.")
          .def_readwrite("default_focal_length_factor", &IROpts::default_focal_length_factor,
                         "If camera parameters are not specified manually and the image does not "
                         "have focal length EXIF information, the focal length is set to the "
                         "value `default_focal_length_factor * max(width, height)`.")
          .def_readwrite("camera_mask_path", &IROpts::camera_mask_path,
                         "Optional path to an image file specifying a mask for all images. No "
                         "features will be extracted in regions where the mask is black (pixel "
                         "intensity value 0 in grayscale)");
    make_dataclass(PyImageReaderOptions);
    auto reader_options = PyImageReaderOptions().cast<IROpts>();

    using SEOpts = SiftExtractionOptions;
    auto PyNormalization = py::enum_<SEOpts::Normalization>(m, "Normalization")
        .value("L1_ROOT", SEOpts::Normalization::L1_ROOT,
               "L1-normalizes each descriptor followed by element-wise square rooting. "
               "This normalization is usually better than standard L2-normalization. "
               "See 'Three things everyone should know to improve object retrieval', "
               "Relja Arandjelovic and Andrew Zisserman, CVPR 2012.")
        .value("L2", SEOpts::Normalization::L2, "Each vector is L2-normalized.");
    AddStringToEnumConstructor(PyNormalization);
    auto PySiftExtractionOptions =
        py::class_<SEOpts>(m, "SiftExtractionOptions")
          .def(py::init<>())
          .def_readwrite("num_threads", &SEOpts::num_threads,
                         "Number of threads for feature matching and geometric verification.")
          .def_readwrite("gpu_index", &SEOpts::gpu_index,
                         "Index of the GPU used for feature matching. For multi-GPU matching, "
                         "you should separate multiple GPU indices by comma, e.g., '0,1,2,3'.")
          .def_readwrite("max_image_size", &SEOpts::max_image_size,
                         "Maximum image size, otherwise image will be down-scaled.")
          .def_readwrite("max_num_features", &SEOpts::max_num_features,
                         "Maximum number of features to detect, keeping larger-scale features.")
          .def_readwrite("max_num_features", &SEOpts::max_num_features,
                         "Maximum number of features to detect, keeping larger-scale features.")
          .def_readwrite("first_octave", &SEOpts::first_octave,
                         "First octave in the pyramid, i.e. -1 upsamples the image by one level.")
          .def_readwrite("num_octaves", &SEOpts::num_octaves)
          .def_readwrite("octave_resolution", &SEOpts::octave_resolution,
                         "Number of levels per octave.")
          .def_readwrite("peak_threshold", &SEOpts::peak_threshold,
                         "Peak threshold for detection.")
          .def_readwrite("edge_threshold", &SEOpts::edge_threshold,
                         "Edge threshold for detection.")
          .def_readwrite("estimate_affine_shape", &SEOpts::estimate_affine_shape,
                         "Estimate affine shape of SIFT features in the form of oriented ellipses as "
                         "opposed to original SIFT which estimates oriented disks.")
          .def_readwrite("max_num_orientations", &SEOpts::max_num_orientations,
                         "Maximum number of orientations per keypoint if not estimate_affine_shape.")
          .def_readwrite("upright", &SEOpts::upright,
                         "Fix the orientation to 0 for upright features")
          .def_readwrite("darkness_adaptivity", &SEOpts::darkness_adaptivity,
                         "Whether to adapt the feature detection depending on the image darkness. "
                         "only available on GPU.")
          .def_readwrite("domain_size_pooling", &SEOpts::domain_size_pooling,
                         "\"Domain-Size Pooling in Local Descriptors and Network"
                         "Architectures\", J. Dong and S. Soatto, CVPR 2015")
          .def_readwrite("dsp_min_scale", &SEOpts::dsp_min_scale)
          .def_readwrite("dsp_max_scale", &SEOpts::dsp_max_scale)
          .def_readwrite("dsp_num_scales", &SEOpts::dsp_num_scales)
          .def_readwrite("normalization", &SEOpts::normalization,
                         "L1_ROOT or L2 descriptor normalization");
    make_dataclass(PySiftExtractionOptions);
    auto sift_extraction_options = PySiftExtractionOptions().cast<SEOpts>();

    using SMOpts = SiftMatchingOptions;
    auto PySiftMatchingOptions =
        py::class_<SMOpts>(m, "SiftMatchingOptions")
          .def(py::init<>())
          .def_readwrite("num_threads",&SMOpts::num_threads)
          .def_readwrite("gpu_index",&SMOpts::gpu_index,
                         "Index of the GPU used for feature matching. For multi-GPU matching, "
                         "you should separate multiple GPU indices by comma, e.g., \"0,1,2,3\".")
          .def_readwrite("max_ratio",&SMOpts::max_ratio,
                         "Maximum distance ratio between first and second best match.")
          .def_readwrite("max_distance",&SMOpts::max_distance,
                         "Maximum distance to best match.")
          .def_readwrite("cross_check",&SMOpts::cross_check,
                         "Whether to enable cross checking in matching.")
          .def_readwrite("max_num_matches",&SMOpts::max_num_matches,
                         "Maximum number of matches.")
          .def_readwrite("max_error",&SMOpts::max_error,
                         "Maximum epipolar error in pixels for geometric verification.")
          .def_readwrite("confidence",&SMOpts::confidence,
                         "Confidence threshold for geometric verification.")
          .def_readwrite("min_num_trials",&SMOpts::min_num_trials,
                         "Minimum number of RANSAC iterations. Note that this option "
                         "overrules the min_inlier_ratio option.")
          .def_readwrite("max_num_trials",&SMOpts::max_num_trials,
                         "Maximum number of RANSAC iterations. Note that this option "
                         "overrules the min_inlier_ratio option.")
          .def_readwrite("min_inlier_ratio",&SMOpts::min_inlier_ratio,
                         "A priori assumed minimum inlier ratio, which determines the maximum "
                         "number of iterations.")
          .def_readwrite("min_num_inliers",&SMOpts::min_num_inliers,
                         "Minimum number of inliers for an image pair to be considered as "
                         "geometrically verified.")
          .def_readwrite("multiple_models",&SMOpts::multiple_models,
                         "Whether to attempt to estimate multiple geometric models per image pair.")
          .def_readwrite("guided_matching",&SMOpts::guided_matching,
                         "Whether to perform guided matching, if geometric verification succeeds.")
          .def_readwrite("planar_scene",&SMOpts::planar_scene,
                         "Force Homography use for Two-view Geometry (can help for planar scenes)");
    make_dataclass(PySiftMatchingOptions);
    auto sift_matching_options = PySiftMatchingOptions().cast<SMOpts>();


    using SeqMOpts = SequentialMatchingOptions;
    auto PySequentialMatchingOptions =
        py::class_<SeqMOpts>(m, "SequentialMatchingOptions")
          .def(py::init<>())
          .def_readwrite("overlap", &SeqMOpts::overlap,
                         "Number of overlapping image pairs.")
          .def_readwrite("quadratic_overlap", &SeqMOpts::quadratic_overlap,
                         "Whether to match images against their quadratic neighbors.")
          .def_readwrite("loop_detection", &SeqMOpts::loop_detection,
                         "Loop detection is invoked every `loop_detection_period` images.")
          .def_readwrite("loop_detection_num_images", &SeqMOpts::loop_detection_num_images,
                         "The number of images to retrieve in loop detection. This number should "
                         "be significantly bigger than the sequential matching overlap.")
          .def_readwrite("loop_detection_num_nearest_neighbors", &SeqMOpts::loop_detection_num_nearest_neighbors,
                         "Number of nearest neighbors to retrieve per query feature.")
          .def_readwrite("loop_detection_num_checks", &SeqMOpts::loop_detection_num_checks,
                         "Number of nearest-neighbor checks to use in retrieval.")
          .def_readwrite("loop_detection_num_images_after_verification",
                         &SeqMOpts::loop_detection_num_images_after_verification,
                         "How many images to return after spatial verification. Set to 0 to turn off "
                         "spatial verification.")
          .def_readwrite("loop_detection_max_num_features", &SeqMOpts::loop_detection_max_num_features,
                         "The maximum number of features to use for indexing an image. If an "
                         "image has more features, only the largest-scale features will be indexed.")
          .def_readwrite("vocab_tree_path", &SeqMOpts::vocab_tree_path,
                         "Path to the vocabulary tree.");
    make_dataclass(PySequentialMatchingOptions);
    auto sequential_options = PySequentialMatchingOptions().cast<SeqMOpts>();

    using SpMOpts = SpatialMatchingOptions;
    auto PySpatialMatchingOptions =
        py::class_<SpMOpts>(m, "SpatialMatchingOptions")
          .def(py::init<>())
          .def_readwrite("is_gps", &SpMOpts::is_gps,
                         "Whether the location priors in the database are GPS coordinates in "
                         "the form of longitude and latitude coordinates in degrees.")
          .def_readwrite("ignore_z", &SpMOpts::ignore_z,
                         "Whether to ignore the Z-component of the location prior.")
          .def_readwrite("max_num_neighbors", &SpMOpts::max_num_neighbors,
                         "The maximum number of nearest neighbors to match.")
          .def_readwrite("max_distance", &SpMOpts::max_distance,
                         "The maximum distance between the query and nearest neighbor [meters].");
    make_dataclass(PySpatialMatchingOptions);
    auto spatial_options = PySpatialMatchingOptions().cast<SpMOpts>();

    using VTMOpts = VocabTreeMatchingOptions;
    auto PyVocabTreeMatchingOptions =
        py::class_<VTMOpts>(m, "VocabTreeMatchingOptions")
          .def(py::init<>())
          .def_readwrite("num_images", &VTMOpts::num_images,
                         "Number of images to retrieve for each query image.")
          .def_readwrite("num_nearest_neighbors", &VTMOpts::num_nearest_neighbors,
                         "Number of nearest neighbors to retrieve per query feature.")
          .def_readwrite("num_checks", &VTMOpts::num_checks,
                         "Number of nearest-neighbor checks to use in retrieval.")
          .def_readwrite("num_images_after_verification", &VTMOpts::num_images_after_verification,
                         "How many images to return after spatial verification. Set to 0 to turn off "
                         "spatial verification.")
          .def_readwrite("max_num_features", &VTMOpts::max_num_features,
                         "The maximum number of features to use for indexing an image.")
          .def_readwrite("vocab_tree_path", &VTMOpts::vocab_tree_path,
                         "Path to the vocabulary tree.")
          .def_readwrite("match_list_path", &VTMOpts::match_list_path,
                         "Optional path to file with specific image names to match.")
          .def("check", [](VTMOpts& self) {
            THROW_CHECK_MSG(!self.vocab_tree_path.empty(), "vocab_tree_path required.");
            THROW_CHECK_FILE_OPEN(self.vocab_tree_path);
          });
    make_dataclass(PyVocabTreeMatchingOptions);
    auto vocabtree_options = PyVocabTreeMatchingOptions().cast<VTMOpts>();

    /* PIPELINE */
    m.def("extract_features", &extract_features,
          py::arg("database_path"),
          py::arg("image_path"),
          py::arg("image_list") = std::vector<std::string>(),
          py::arg("camera_mode") = CameraMode::AUTO,
          py::arg("camera_model") = "SIMPLE_RADIAL",
          py::arg("reader_options") = reader_options,
          py::arg("sift_options") = sift_extraction_options,
          py::arg("device") = Device::AUTO,
          py::arg("verbose") = true,
          "Extract SIFT Features and write them to database"
    );

    m.def("match_exhaustive", &match_exhaustive,
          py::arg("database_path"),
          py::arg("sift_options") = sift_matching_options,
          py::arg("block_size") = 50,
          py::arg("device") = Device::AUTO,
          py::arg("verbose") = true,
          "Exhaustive feature matching");

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

    m.def("import_images",
          &import_images,
          py::arg("database_path"),
          py::arg("image_path"),
          py::arg("camera_mode") = CameraMode::AUTO,
          py::arg("camera_model") = std::string(),
          py::arg("image_list") = std::vector<std::string>(),
          "Import images into a database");

    m.def("verify_matches",
          &verify_matches,
          py::arg("database_path"),
          py::arg("pairs_path"),
          py::arg("max_num_trials") = SiftMatchingOptions().max_num_trials,
          py::arg("min_inlier_ratio") = SiftMatchingOptions().min_inlier_ratio,
          "Run geometric verification of the matches");

    using Opts = IncrementalMapperOptions;
    auto PyIncrementalMapperOptions =
        py::class_<Opts>(m, "IncrementalMapperOptions")
            .def(py::init<>())
            .def_readwrite("min_num_matches", &Opts::min_num_matches)
            .def_readwrite("ignore_watermarks", &Opts::ignore_watermarks)
            .def_readwrite("multiple_models", &Opts::multiple_models)
            .def_readwrite("max_num_models", &Opts::max_num_models)
            .def_readwrite("max_model_overlap", &Opts::max_model_overlap)
            .def_readwrite("min_model_size", &Opts::min_model_size)
            .def_readwrite("init_image_id1", &Opts::init_image_id1)
            .def_readwrite("init_image_id2", &Opts::init_image_id2)
            .def_readwrite("init_num_trials", &Opts::init_num_trials)
            .def_readwrite("extract_colors", &Opts::extract_colors)
            .def_readwrite("num_threads", &Opts::num_threads)
            .def_readwrite("min_focal_length_ratio",
                           &Opts::min_focal_length_ratio)
            .def_readwrite("max_focal_length_ratio",
                           &Opts::max_focal_length_ratio)
            .def_readwrite("max_extra_param", &Opts::max_extra_param)
            .def_readwrite("ba_refine_focal_length",
                           &Opts::ba_refine_focal_length)
            .def_readwrite("ba_refine_principal_point",
                           &Opts::ba_refine_principal_point)
            .def_readwrite("ba_refine_extra_params",
                           &Opts::ba_refine_extra_params)
            .def_readwrite("ba_min_num_residuals_for_multi_threading",
                           &Opts::ba_min_num_residuals_for_multi_threading)
            .def_readwrite("ba_local_num_images", &Opts::ba_local_num_images)
            .def_readwrite("ba_local_function_tolerance",
                           &Opts::ba_local_function_tolerance)
            .def_readwrite("ba_local_max_num_iterations",
                           &Opts::ba_local_max_num_iterations)
            .def_readwrite("ba_global_use_pba", &Opts::ba_global_use_pba)
            .def_readwrite("ba_global_pba_gpu_index",
                           &Opts::ba_global_pba_gpu_index)
            .def_readwrite("ba_global_images_ratio",
                           &Opts::ba_global_images_ratio)
            .def_readwrite("ba_global_points_ratio",
                           &Opts::ba_global_points_ratio)
            .def_readwrite("ba_global_images_freq",
                           &Opts::ba_global_images_freq)
            .def_readwrite("ba_global_points_freq",
                           &Opts::ba_global_points_freq)
            .def_readwrite("ba_global_function_tolerance",
                           &Opts::ba_global_function_tolerance)
            .def_readwrite("ba_global_max_num_iterations",
                           &Opts::ba_global_max_num_iterations)
            .def_readwrite("ba_local_max_refinements",
                           &Opts::ba_local_max_refinements)
            .def_readwrite("ba_local_max_refinement_change",
                           &Opts::ba_local_max_refinement_change)
            .def_readwrite("ba_global_max_refinements",
                           &Opts::ba_global_max_refinements)
            .def_readwrite("ba_global_max_refinement_change",
                           &Opts::ba_global_max_refinement_change)
            .def_readwrite("snapshot_path", &Opts::snapshot_path)
            .def_readwrite("snapshot_images_freq", &Opts::snapshot_images_freq)
            .def_readwrite("image_names", &Opts::image_names)
            .def_readwrite("fix_existing_images", &Opts::fix_existing_images);
    make_dataclass(PyIncrementalMapperOptions);
    auto mapper_options = PyIncrementalMapperOptions().cast<Opts>();

    m.def("triangulate_points",
          &triangulate_points,
          py::arg("reconstruction"),
          py::arg("database_path"),
          py::arg("image_path"),
          py::arg("output_path"),
          py::arg("clear_points") = true,
          py::arg("options") = mapper_options,
          "Triangulate 3D points from known camera poses");

    m.def("incremental_mapping",
          static_cast<std::map<size_t, Reconstruction> (*)(
              const py::object,
              const py::object,
              const py::object,
              const IncrementalMapperOptions&)>(&incremental_mapping),
          py::arg("database_path"),
          py::arg("image_path"),
          py::arg("output_path"),
          py::arg("options") = mapper_options,
          "Triangulate 3D points from known poses");

    m.def("incremental_mapping",
          static_cast<std::map<size_t, Reconstruction> (*)(const py::object,
                                                           const py::object,
                                                           const py::object,
                                                           const int,
                                                           const int)>(
              &incremental_mapping),
          py::arg("database_path"),
          py::arg("image_path"),
          py::arg("output_path"),
          py::arg("num_threads") = mapper_options.num_threads,
          py::arg("min_num_matches") = mapper_options.min_num_matches,
          "Triangulate 3D points from known poses");

    m.def("infer_camera_from_image",
          &infer_camera_from_image,
          py::arg("image_path"),
          "Guess the camera parameters from the EXIF metadata");
}
