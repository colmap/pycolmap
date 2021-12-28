#include "colmap/base/reconstruction.h"
#include "colmap/base/image_reader.h"
#include "colmap/base/camera_models.h"
#include "colmap/util/misc.h"
#include "colmap/feature/sift.h"
#include "colmap/feature/matching.h"
#include "colmap/controllers/incremental_mapper.h"

using namespace colmap;

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/iostream.h>

namespace py = pybind11;
using namespace pybind11::literals;

#include "log_exceptions.h"


// Copied from colmap/exe/features.cc
// TODO: expose these in headers
enum class CameraMode { AUTO = 0, SINGLE = 1, PER_FOLDER = 2, PER_IMAGE = 3 };
void UpdateImageReaderOptionsFromCameraMode(ImageReaderOptions& options,
                                            CameraMode mode) {
  switch (mode) {
    case CameraMode::AUTO:
      options.single_camera = false;
      options.single_camera_per_folder = false;
      options.single_camera_per_image = false;
      break;
    case CameraMode::SINGLE:
      options.single_camera = true;
      options.single_camera_per_folder = false;
      options.single_camera_per_image = false;
      break;
    case CameraMode::PER_FOLDER:
      options.single_camera = false;
      options.single_camera_per_folder = true;
      options.single_camera_per_image = false;
      break;
    case CameraMode::PER_IMAGE:
      options.single_camera = false;
      options.single_camera_per_folder = false;
      options.single_camera_per_image = true;
      break;
  }
}

void import_images(
        const py::object database_path_,
        const py::object image_path_,
        const CameraMode camera_mode,
        const std::string camera_model,
        const std::vector<std::string> image_list
) {
    std::string database_path = py::str(database_path_).cast<std::string>();
    THROW_CUSTOM_CHECK_MSG(
        ExistsFile(database_path),
        std::invalid_argument,
        (std::string("Database file does not exist: ")+ database_path).c_str()
    );
    std::string image_path = py::str(image_path_).cast<std::string>();
    THROW_CUSTOM_CHECK_MSG(
        ExistsDir(image_path),
        std::invalid_argument,
        (std::string("Image directory does not exist: ")+ image_path).c_str()
    );

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
    THROW_CUSTOM_CHECK_MSG(
        ExistsFile(image_path),
        std::invalid_argument,
        (std::string("Image file does not exist: ") + image_path).c_str()
    );

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
    THROW_CUSTOM_CHECK_MSG(
        ExistsFile(database_path),
        std::invalid_argument,
        (std::string("Database file does not exist: ")+ database_path).c_str()
    );
    std::string pairs_path = py::str(pairs_path_).cast<std::string>();
    THROW_CUSTOM_CHECK_MSG(
        ExistsFile(pairs_path),
        std::invalid_argument,
        (std::string("List of image pairs does not exist: ")+ database_path).c_str()
    );

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

// Copied from colmap/exe/sfm.cc
// TODO: split options vs body and expose in header
Reconstruction RunPointTriangulator(
    Reconstruction reconstruction,
    const std::string database_path,
    const std::string image_path,
    const std::string output_path,
    IncrementalMapperOptions mapper_options,
    const bool clear_points = true
) {
    DatabaseCache database_cache;
    {
        Database database(database_path);

        const size_t min_num_matches =
            static_cast<size_t>(mapper_options.min_num_matches);
        database_cache.Load(database, min_num_matches,
                            mapper_options.ignore_watermarks,
                            mapper_options.image_names);

        if (clear_points) {
            reconstruction.DeleteAllPoints2DAndPoints3D();
            reconstruction.TranscribeImageIdsToDatabase(database);
        }
    }

    CHECK_GE(reconstruction.NumRegImages(), 2)
        << "Need at least two images for triangulation";

    IncrementalMapper mapper(&database_cache);
    mapper.BeginReconstruction(&reconstruction);

    //////////////////////////////////////////////////////////////////////////////
    // Triangulation
    //////////////////////////////////////////////////////////////////////////////
    const auto tri_options = mapper_options.Triangulation();
    const auto& reg_image_ids = reconstruction.RegImageIds();
    for (size_t i = 0; i < reg_image_ids.size(); ++i) {
        const image_t image_id = reg_image_ids[i];
        const auto& image = reconstruction.Image(image_id);
        //PrintHeading1(StringPrintf("Triangulating image #%d (%d)", image_id, i));
        const size_t num_existing_points3D = image.NumPoints3D();
        //std::cout << "  => Image sees " << num_existing_points3D << " / "
                  //<< image.NumObservations() << " points" << std::endl;
        mapper.TriangulateImage(tri_options, image_id);
        //std::cout << "  => Triangulated "
                  //<< (image.NumPoints3D() - num_existing_points3D) << " points"
                  //<< std::endl;
    }

    //////////////////////////////////////////////////////////////////////////////
    // Retriangulation
    //////////////////////////////////////////////////////////////////////////////
    PrintHeading1("Retriangulation");
    CompleteAndMergeTracks(mapper_options, &mapper);

    //////////////////////////////////////////////////////////////////////////////
    // Bundle adjustment
    //////////////////////////////////////////////////////////////////////////////
    auto ba_options = mapper_options.GlobalBundleAdjustment();
    ba_options.refine_focal_length = false;
    ba_options.refine_principal_point = false;
    ba_options.refine_extra_params = false;
    ba_options.refine_extrinsics = false;
    ba_options.print_summary = false;

    // Configure bundle adjustment.
    BundleAdjustmentConfig ba_config;
    for (const image_t image_id : reconstruction.RegImageIds()) {
        ba_config.AddImage(image_id);
    }

    for (int i = 0; i < mapper_options.ba_global_max_refinements; ++i) {
        // Avoid degeneracies in bundle adjustment.
        reconstruction.FilterObservationsWithNegativeDepth();

        const size_t num_observations = reconstruction.ComputeNumObservations();

        //PrintHeading1("Bundle adjustment");
        BundleAdjuster bundle_adjuster(ba_options, ba_config);
        CHECK(bundle_adjuster.Solve(&reconstruction));

        size_t num_changed_observations = 0;
        num_changed_observations += CompleteAndMergeTracks(mapper_options, &mapper);
        num_changed_observations += FilterPoints(mapper_options, &mapper);
        const double changed =
        static_cast<double>(num_changed_observations) / num_observations;
        std::cout << StringPrintf("  => Changed observations: %.6f", changed)
                  << std::endl;
        if (changed < mapper_options.ba_global_max_refinement_change) {
            break;
        }
    }

    PrintHeading1("Extracting colors");
    reconstruction.ExtractColorsForAllImages(image_path);

    const bool kDiscardReconstruction = false;
    mapper.EndReconstruction(kDiscardReconstruction);
    reconstruction.Write(output_path);
    return reconstruction;
}

Reconstruction triangulate_points(
        Reconstruction reconstruction,
        const py::object database_path_,
        const py::object image_path_,
        const py::object output_path_,
        const bool clear_points
) {
    std::string database_path = py::str(database_path_).cast<std::string>();
    THROW_CUSTOM_CHECK_MSG(
        ExistsFile(database_path),
        std::invalid_argument,
        (std::string("Database file does not exist: ")+ database_path).c_str()
    );
    std::string image_path = py::str(image_path_).cast<std::string>();
    THROW_CUSTOM_CHECK_MSG(
        ExistsDir(image_path),
        std::invalid_argument,
        (std::string("Image directory does not exist: ")+ image_path).c_str()
    );
    std::string output_path = py::str(output_path_).cast<std::string>();
    CreateDirIfNotExists(output_path);

    IncrementalMapperOptions mapper_options;
    return RunPointTriangulator(
        reconstruction, database_path, image_path, output_path,
        mapper_options, clear_points);

}

// Copied from colmap/exe/sfm.cc
std::map<size_t, Reconstruction> incremental_mapping(
        const py::object database_path_,
        const py::object image_path_,
        const py::object output_path_,
        const int num_threads,
        const int min_num_matches
) {
    std::string database_path = py::str(database_path_).cast<std::string>();
    THROW_CUSTOM_CHECK_MSG(
        ExistsFile(database_path),
        std::invalid_argument,
        (std::string("Database file does not exist: ")+ database_path).c_str()
    );
    std::string image_path = py::str(image_path_).cast<std::string>();
    THROW_CUSTOM_CHECK_MSG(
        ExistsDir(image_path),
        std::invalid_argument,
        (std::string("Image directory does not exist: ")+ image_path).c_str()
    );
    std::string output_path = py::str(output_path_).cast<std::string>();
    CreateDirIfNotExists(output_path);

    IncrementalMapperOptions options;
    options.num_threads = num_threads;
    options.min_num_matches = min_num_matches;
    ReconstructionManager reconstruction_manager;
    IncrementalMapperController mapper(&options, image_path, database_path,
                                       &reconstruction_manager);

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


void init_pipeline(py::module& m) {
    py::enum_<CameraMode>(m, "CameraMode")
        .value("AUTO", CameraMode::AUTO)
        .value("SINGLE", CameraMode::SINGLE)
        .value("PER_FOLDER", CameraMode::PER_FOLDER)
        .value("PER_IMAGE", CameraMode::PER_IMAGE);

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

    m.def("triangulate_points",
          &triangulate_points,
          py::arg("reconstruction"),
          py::arg("database_path"),
          py::arg("image_path"),
          py::arg("output_path"),
          py::arg("clear_points") = true,
          "Triangulate 3D points from known camera poses");

    m.def("incremental_mapping",
          &incremental_mapping,
          py::arg("database_path"),
          py::arg("image_path"),
          py::arg("output_path"),
          py::arg("num_threads") = IncrementalMapperOptions().num_threads,
          py::arg("min_num_matches") = IncrementalMapperOptions().min_num_matches,
          "Triangulate 3D points from known poses");

    m.def("infer_camera_from_image",
          &infer_camera_from_image,
          py::arg("image_path"),
          "Guess the camera parameters from the EXIF metadata");
}
