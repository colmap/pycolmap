// Author: Paul-Edouard Sarlin (skydes)

#include "colmap/base/camera_models.h"
#include "colmap/base/image_reader.h"
#include "colmap/base/reconstruction.h"
#include "colmap/base/undistortion.h"
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

void import_images(const py::object database_path_,
                   const py::object image_path_,
                   const CameraMode camera_mode,
                   const std::vector<std::string> image_list,
                   const ImageReaderOptions options_) {
    std::string database_path = py::str(database_path_).cast<std::string>();
    THROW_CHECK_FILE_EXISTS(database_path);
    std::string image_path = py::str(image_path_).cast<std::string>();
    THROW_CHECK_DIR_EXISTS(image_path);

    ImageReaderOptions options(options_);
    options.database_path = database_path;
    options.image_path = image_path;
    options.image_list = image_list;
    UpdateImageReaderOptionsFromCameraMode(options, camera_mode);
    THROW_CUSTOM_CHECK_MSG(
        ExistsCameraModelWithName(options.camera_model),
        std::invalid_argument,
        (std::string("Invalid camera model: ") + options.camera_model).c_str());

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

void import_images(const py::object database_path_,
                   const py::object image_path_,
                   const CameraMode camera_mode,
                   const std::string camera_model,
                   const std::vector<std::string> image_list) {
    ImageReaderOptions options;
    if (!camera_model.empty()) {
        options.camera_model = camera_model;
    }
    return import_images(
        database_path_, image_path_, camera_mode, image_list, options);
}

Camera infer_camera_from_image(const py::object image_path_,
                               const ImageReaderOptions options) {
    std::string image_path = py::str(image_path_).cast<std::string>();
    THROW_CHECK_FILE_EXISTS(image_path);

    Bitmap bitmap;
    THROW_CUSTOM_CHECK_MSG(
        bitmap.Read(image_path, false),
        std::invalid_argument,
        (std::string("Cannot read image file: ") + image_path).c_str());

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
    camera.InitializeWithId(
        camera.ModelId(), focal_length, bitmap.Width(), bitmap.Height());
    THROW_CUSTOM_CHECK_MSG(
        camera.VerifyParams(),
        std::invalid_argument,
        (std::string("Invalid camera params: ") + camera.ParamsToString())
            .c_str());

    return camera;
}

void undistort_images(py::object output_path_,
                      py::object input_path_,
                      py::object image_path_,
                      std::vector<std::string> image_list,
                      std::string output_type,
                      CopyType copy_type,
                      int num_patch_match_src_images,
                      UndistortCameraOptions undistort_camera_options,
                      bool verbose) {
    std::string output_path = py::str(output_path_).cast<std::string>();
    std::string input_path = py::str(input_path_).cast<std::string>();
    THROW_CHECK_DIR_EXISTS(input_path);
    std::string image_path = py::str(image_path_).cast<std::string>();
    THROW_CHECK_DIR_EXISTS(image_path);

    CreateDirIfNotExists(output_path);
    if (verbose) {
        PrintHeading1("Reading reconstruction");
    }
    Reconstruction reconstruction;
    reconstruction.Read(input_path);
    if (verbose) {
        std::cout << StringPrintf(
                         " => Reconstruction with %d images and %d points",
                         reconstruction.NumImages(),
                         reconstruction.NumPoints3D())
                  << std::endl;
    }

    std::vector<image_t> image_ids;
    for (const auto& image_name : image_list) {
        const Image* image = reconstruction.FindImageWithName(image_name);
        if (image != nullptr) {
            image_ids.push_back(image->ImageId());
        } else {
            std::cout << "WARN: Cannot find image " << image_name << std::endl;
        }
    }

    std::unique_ptr<Thread> undistorter;
    if (output_type == "COLMAP") {
        undistorter.reset(new COLMAPUndistorter(undistort_camera_options,
                                                reconstruction,
                                                image_path,
                                                output_path,
                                                num_patch_match_src_images,
                                                copy_type,
                                                image_ids));
    } else if (output_type == "PMVS") {
        undistorter.reset(new PMVSUndistorter(
            undistort_camera_options, reconstruction, image_path, output_path));
    } else if (output_type == "CMP-MVS") {
        undistorter.reset(new CMPMVSUndistorter(
            undistort_camera_options, reconstruction, image_path, output_path));
    } else {
        THROW_EXCEPTION(
            std::invalid_argument,
            std::string("Invalid `output_type` - supported values are ") +
                "{'COLMAP', 'PMVS', 'CMP-MVS'}.");
    }

    std::stringstream oss;
    std::streambuf* oldcout = nullptr;
    if (!verbose) {
        oldcout = std::cout.rdbuf(oss.rdbuf());
    }

    py::gil_scoped_release release;
    undistorter->Start();
    PyWait(undistorter.get());

    if (!verbose) {
        std::cout.rdbuf(oldcout);
    }
}

void init_images(py::module& m) {
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
            .def_readwrite("camera_model",
                           &IROpts::camera_model,
                           "Name of the camera model.")
            .def_readwrite("mask_path",
                           &IROpts::mask_path,
                           "Optional root path to folder which contains image"
                           "masks. For a given image, the corresponding mask"
                           "must have the same sub-path below this root as the"
                           "image has below image_path. The filename must be"
                           "equal, aside from the added extension .png. "
                           "For example, for an image image_path/abc/012.jpg,"
                           "the mask would be mask_path/abc/012.jpg.png. No"
                           "features will be extracted in regions where the"
                           "mask image is black (pixel intensity value 0 in"
                           "grayscale).")
            .def_readwrite("existing_camera_id",
                           &IROpts::existing_camera_id,
                           "Whether to explicitly use an existing camera for "
                           "all images. Note that in this case the specified "
                           "camera model and parameters are ignored.")
            .def_readwrite("camera_params",
                           &IROpts::camera_params,
                           "Manual specification of camera parameters. If "
                           "empty, camera parameters will be extracted from "
                           "EXIF, i.e. principal point and focal length.")
            .def_readwrite(
                "default_focal_length_factor",
                &IROpts::default_focal_length_factor,
                "If camera parameters are not specified manually and the image "
                "does not have focal length EXIF information, the focal length "
                "is set to the value `default_focal_length_factor * max(width, "
                "height)`.")
            .def_readwrite(
                "camera_mask_path",
                &IROpts::camera_mask_path,
                "Optional path to an image file specifying a mask for all "
                "images. No features will be extracted in regions where the "
                "mask is black (pixel intensity value 0 in grayscale)");
    make_dataclass(PyImageReaderOptions);
    auto reader_options = PyImageReaderOptions().cast<IROpts>();

    auto PyCopyType = py::enum_<CopyType>(m, "CopyType")
                          .value("copy", CopyType::COPY)
                          .value("soft-link", CopyType::SOFT_LINK)
                          .value("hard-link", CopyType::HARD_LINK);
    AddStringToEnumConstructor(PyCopyType);

    using UDOpts = UndistortCameraOptions;
    auto PyUndistortCameraOptions =
        py::class_<UDOpts>(m, "UndistortCameraOptions")
            .def(py::init<>())
            .def_readwrite("blank_pixels",
                           &UDOpts::blank_pixels,
                           "The amount of blank pixels in the undistorted "
                           "image in the range [0, 1].")
            .def_readwrite("min_scale",
                           &UDOpts::min_scale,
                           "Minimum scale change of camera used to satisfy the "
                           "blank pixel constraint.")
            .def_readwrite("max_scale",
                           &UDOpts::max_scale,
                           "Maximum scale change of camera used to satisfy the "
                           "blank pixel constraint.")
            .def_readwrite("max_image_size",
                           &UDOpts::max_image_size,
                           "Maximum image size in terms of width or height of "
                           "the undistorted camera.")
            .def_readwrite("roi_min_x", &UDOpts::roi_min_x)
            .def_readwrite("roi_min_y", &UDOpts::roi_min_y)
            .def_readwrite("roi_max_x", &UDOpts::roi_max_x)
            .def_readwrite("roi_max_y", &UDOpts::roi_max_y);
    make_dataclass(PyUndistortCameraOptions);
    auto undistort_options = PyUndistortCameraOptions().cast<UDOpts>();

    m.def(
        "import_images",
        static_cast<void (*)(const py::object,
                             const py::object,
                             const CameraMode,
                             const std::vector<std::string>,
                             const ImageReaderOptions
                             )>(&import_images),
        py::arg("database_path"),
        py::arg("image_path"),
        py::arg("camera_mode") = CameraMode::AUTO,
        py::arg("image_list") = std::vector<std::string>(),
        py::arg("options") = reader_options,
        "Import images into a database");

    m.def(
        "import_images",
        static_cast<void (*)(const py::object,
                             const py::object,
                             const CameraMode,
                             const std::string,
                             const std::vector<std::string>
                             )>(&import_images),
        py::arg("database_path"),
        py::arg("image_path"),
        py::arg("camera_mode") = CameraMode::AUTO,
        py::arg("camera_model") = std::string(),
        py::arg("image_list") = std::vector<std::string>(),
        "Import images into a database");

    m.def("infer_camera_from_image",
          &infer_camera_from_image,
          py::arg("image_path"),
          py::arg("options") = reader_options,
          "Guess the camera parameters from the EXIF metadata");

    m.def("undistort_images",
          &undistort_images,
          py::arg("output_path"),
          py::arg("input_path"),
          py::arg("image_path"),
          py::arg("image_list") = std::vector<std::string>(),
          py::arg("output_type") = "COLMAP",
          py::arg("copy_policy") = CopyType::COPY,
          py::arg("num_patch_match_src_images") = 20,
          py::arg("undistort_options") = undistort_options,
          py::arg("verbose") = false,
          "Undistort images");
}
