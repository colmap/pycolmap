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

    m.def("import_images",
          &import_images,
          py::arg("database_path"),
          py::arg("image_path"),
          py::arg("camera_mode") = CameraMode::AUTO,
          py::arg("camera_model") = std::string(),
          py::arg("image_list") = std::vector<std::string>(),
          "Import images into a database");

    m.def("infer_camera_from_image",
          &infer_camera_from_image,
          py::arg("image_path"),
          "Guess the camera parameters from the EXIF metadata");
}
