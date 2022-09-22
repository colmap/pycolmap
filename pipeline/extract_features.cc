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

void VerifyCameraParams(const std::string& camera_model,
                        const std::string& params) {
    THROW_CUSTOM_CHECK_MSG(
        ExistsCameraModelWithName(camera_model),
        std::invalid_argument,
        (std::string("Invalid camera model: ") + camera_model).c_str());

    const std::vector<double> camera_params = CSVToVector<double>(params);
    const int camera_model_id = CameraModelNameToId(camera_model);

    if (camera_params.size() > 0 &&
        !CameraModelVerifyParams(camera_model_id, camera_params)) {
        THROW_EXCEPTION(std::invalid_argument, "Invalid camera parameters.");
    }
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
    THROW_CHECK_MSG(!ExistsFile(database_path),
                    database_path + " already exists.");
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
        oldcout = std::cout.rdbuf(oss.rdbuf());
    }
    py::gil_scoped_release release;
    SiftFeatureExtractor feature_extractor(reader_options, sift_options);

    feature_extractor.Start();
    PyWait(&feature_extractor);

    if (!verbose) {
        std::cout.rdbuf(oldcout);
    }
}

void init_extract_features(py::module& m) {
    /* OPTIONS */
    using SEOpts = SiftExtractionOptions;
    auto PyNormalization =
        py::enum_<SEOpts::Normalization>(m, "Normalization")
            .value("L1_ROOT",
                   SEOpts::Normalization::L1_ROOT,
                   "L1-normalizes each descriptor followed by element-wise "
                   "square rooting. This normalization is usually better than "
                   "standard "
                   "L2-normalization. See 'Three things everyone should know "
                   "to improve object retrieval', Relja Arandjelovic and "
                   "Andrew Zisserman, CVPR 2012.")
            .value("L2",
                   SEOpts::Normalization::L2,
                   "Each vector is L2-normalized.");
    AddStringToEnumConstructor(PyNormalization);
    auto PySiftExtractionOptions =
        py::class_<SEOpts>(m, "SiftExtractionOptions")
            .def(py::init<>())
            .def_readwrite("num_threads",
                           &SEOpts::num_threads,
                           "Number of threads for feature matching and "
                           "geometric verification.")
            .def_readwrite("gpu_index",
                           &SEOpts::gpu_index,
                           "Index of the GPU used for feature matching. For "
                           "multi-GPU matching, you should separate multiple "
                           "GPU indices by comma, e.g., '0,1,2,3'.")
            .def_readwrite(
                "max_image_size",
                &SEOpts::max_image_size,
                "Maximum image size, otherwise image will be down-scaled.")
            .def_readwrite("max_num_features",
                           &SEOpts::max_num_features,
                           "Maximum number of features to detect, keeping "
                           "larger-scale features.")
            .def_readwrite("first_octave",
                           &SEOpts::first_octave,
                           "First octave in the pyramid, i.e. -1 upsamples the "
                           "image by one level.")
            .def_readwrite("num_octaves", &SEOpts::num_octaves)
            .def_readwrite("octave_resolution",
                           &SEOpts::octave_resolution,
                           "Number of levels per octave.")
            .def_readwrite("peak_threshold",
                           &SEOpts::peak_threshold,
                           "Peak threshold for detection.")
            .def_readwrite("edge_threshold",
                           &SEOpts::edge_threshold,
                           "Edge threshold for detection.")
            .def_readwrite("estimate_affine_shape",
                           &SEOpts::estimate_affine_shape,
                           "Estimate affine shape of SIFT features in the form "
                           "of oriented ellipses as opposed to original SIFT "
                           "which estimates oriented disks.")
            .def_readwrite("max_num_orientations",
                           &SEOpts::max_num_orientations,
                           "Maximum number of orientations per keypoint if not "
                           "estimate_affine_shape.")
            .def_readwrite("upright",
                           &SEOpts::upright,
                           "Fix the orientation to 0 for upright features")
            .def_readwrite("darkness_adaptivity",
                           &SEOpts::darkness_adaptivity,
                           "Whether to adapt the feature detection depending "
                           "on the image darkness. only available on GPU.")
            .def_readwrite(
                "domain_size_pooling",
                &SEOpts::domain_size_pooling,
                "\"Domain-Size Pooling in Local Descriptors and Network"
                "Architectures\", J. Dong and S. Soatto, CVPR 2015")
            .def_readwrite("dsp_min_scale", &SEOpts::dsp_min_scale)
            .def_readwrite("dsp_max_scale", &SEOpts::dsp_max_scale)
            .def_readwrite("dsp_num_scales", &SEOpts::dsp_num_scales)
            .def_readwrite("normalization",
                           &SEOpts::normalization,
                           "L1_ROOT or L2 descriptor normalization");
    make_dataclass(PySiftExtractionOptions);
    auto sift_extraction_options = PySiftExtractionOptions().cast<SEOpts>();

    /* PIPELINE */
    m.def("extract_features",
          &extract_features,
          py::arg("database_path"),
          py::arg("image_path"),
          py::arg("image_list") = std::vector<std::string>(),
          py::arg("camera_mode") = CameraMode::AUTO,
          py::arg("camera_model") = "SIMPLE_RADIAL",
          py::arg("reader_options") = ImageReaderOptions(),
          py::arg("sift_options") = sift_extraction_options,
          py::arg("device") = Device::AUTO,
          py::arg("verbose") = true,
          "Extract SIFT Features and write them to database");
}
