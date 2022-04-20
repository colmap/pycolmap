// Author: Paul-Edouard Sarlin (skydes)

#include "colmap/base/reconstruction.h"
#include "colmap/base/image_reader.h"
#include "colmap/base/camera_models.h"
#include "colmap/util/misc.h"
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

    while (image_reader.NextIndex() < image_reader.NumImages()) {
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
    auto PyCameraMode = py::enum_<CameraMode>(m, "CameraMode")
        .value("AUTO", CameraMode::AUTO)
        .value("SINGLE", CameraMode::SINGLE)
        .value("PER_FOLDER", CameraMode::PER_FOLDER)
        .value("PER_IMAGE", CameraMode::PER_IMAGE);
    AddStringToEnumConstructor(PyCameraMode);

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
