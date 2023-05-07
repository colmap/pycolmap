// Author: Mihai-Dusmanu (mihaidusmanu)

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

#include <Eigen/Core>

extern "C" {
#include <lib/VLFeat/sift.h>
}

#include <colmap/feature/sift.h>
#include <colmap/feature/utils.h>
#include <colmap/lib/SiftGPU/SiftGPU.h>
#ifdef CUDA_ENABLED
#include "GL/glew.h"
#endif

using namespace colmap;

#include "helpers.h"
#include "utils.h"

#include <iostream>

#define kdim 4
#define ddim 128

template <typename dtype>
using pyimage_t =
    Eigen::Matrix<dtype, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

typedef Eigen::Matrix<float, Eigen::Dynamic, ddim, Eigen::RowMajor>
    descriptors_t;
typedef Eigen::Matrix<float, Eigen::Dynamic, kdim, Eigen::RowMajor> keypoints_t;
typedef Eigen::VectorXf scores_t;
typedef std::tuple<keypoints_t, scores_t, descriptors_t> sift_output_t;

static std::map<int, std::unique_ptr<std::mutex>> sift_gpu_mutexes;

class Sift {
   public:
    Sift(SiftExtractionOptions options, Device device)
        : options_(options), use_gpu_(IsGPU(device)) {
        VerifySiftGPUParams(use_gpu_);
        if (use_gpu_) {
#ifdef CUDA_ENABLED
            sift_gpu.reset(new SiftGPU);
            CreateSiftGPUExtractor(options_, sift_gpu.get());
            if (sift_gpu_mutexes.count(sift_gpu->gpu_index) == 0) {
                sift_gpu_mutexes.emplace(
                    sift_gpu->gpu_index,
                    std::unique_ptr<std::mutex>(new std::mutex()));
            }
#endif
        }
    }

    template <typename dtype>
    sift_output_t Extract(Eigen::Ref<const pyimage_t<dtype>> image,
                          bool do_normalize) {
        THROW_CHECK_LE(image.rows(), options_.max_image_size);
        THROW_CHECK_LE(image.cols(), options_.max_image_size);

        descriptors_t descriptors;
        keypoints_t keypoints;
        scores_t scores;

        if (use_gpu_) {
#ifdef CUDA_ENABLED
            // TODO: SiftGPU changes the image inplace --> we copy it for now
            pyimage_t<dtype> image_copy = image;
            std::tie(keypoints, scores, descriptors) = ExtractGPU(image_copy);
#endif
        } else {
            std::tie(keypoints, scores, descriptors) = ExtractCPU(image);
        }
        if (do_normalize) {
            // Save and normalize the descriptors.
            if (options_.normalization ==
                SiftExtractionOptions::Normalization::L2) {
                descriptors = L2NormalizeFeatureDescriptors(descriptors);
            } else if (options_.normalization ==
                       SiftExtractionOptions::Normalization::L1_ROOT) {
                descriptors = L1RootNormalizeFeatureDescriptors(descriptors);
            }
        }

        return std::make_tuple(keypoints, scores, descriptors);
    }
    const SiftExtractionOptions& Options() const { return options_; };
    Device GetDevice() const {
        return (use_gpu_) ? Device::CUDA : Device::CPU;
    };

   private:
#ifdef CUDA_ENABLED
    template <typename dtype>
    sift_output_t ExtractGPU(const pyimage_t<dtype>& image /* [h, w] */) {
        THROW_CHECK_LE(options_.max_image_size, sift_gpu->GetMaxDimension());
        THROW_CHECK(!options_.estimate_affine_shape);
        THROW_CHECK(!options_.domain_size_pooling);

        std::unique_lock<std::mutex> lock(
            *sift_gpu_mutexes[sift_gpu->gpu_index]);

        // Note, that this produces slightly different results than using
        // SiftGPU directly for RGB->GRAY conversion, since it uses different
        // weights.
        int code;
        if (std::is_same<dtype, float>::value) {
            code = sift_gpu->RunSIFT(image.cols(),
                                     image.rows(),
                                     image.data(),
                                     GL_LUMINANCE,
                                     GL_FLOAT);
        } else if (std::is_same<dtype, uint8_t>::value) {
            code = sift_gpu->RunSIFT(image.cols(),
                                     image.rows(),
                                     image.data(),
                                     GL_LUMINANCE,
                                     GL_UNSIGNED_BYTE);
        } else {
            THROW_EXCEPTION(std::runtime_error,
                            "SIFT GPU only support float/byte images.");
        }
        const int kSuccessCode = 1;

        THROW_CHECK_EQ(code, kSuccessCode);

        const size_t num_features =
            static_cast<size_t>(sift_gpu->GetFeatureNum());

        std::vector<SiftKeypoint> keypoints_data(num_features);

        descriptors_t descriptors(num_features, ddim);
        keypoints_t keypoints(num_features, kdim);
        scores_t scores = Eigen::VectorXf::Ones(num_features);

        // Download the extracted keypoints and descriptors.
        sift_gpu->GetFeatureVector(keypoints_data.data(), descriptors.data());

        for (size_t i = 0; i < num_features; ++i) {
            keypoints(i, 0) = keypoints_data[i].x;
            keypoints(i, 1) = keypoints_data[i].y;
            keypoints(i, 2) = keypoints_data[i].s;
            keypoints(i, 3) = keypoints_data[i].o;
        }

        return std::make_tuple(keypoints, scores, descriptors);
    }
#endif

    sift_output_t ExtractCPU(Eigen::Ref<const pyimage_t<float>> image) {
        // Create a new instance of SIFT detector & descriptor.
        VlSiftFilt* sift = vl_sift_new(image.cols(),
                                       image.rows(),
                                       options_.num_octaves,
                                       options_.octave_resolution,
                                       options_.first_octave);
        vl_sift_set_edge_thresh(sift, options_.edge_threshold);
        vl_sift_set_peak_thresh(sift, options_.peak_threshold);

        // Build image pyramid.
        bool is_first_octave = true;
        descriptors_t descriptors;
        keypoints_t keypoints;
        scores_t scores;

        int num_feats = 0;

        while (true) {
            if (is_first_octave) {
                if (vl_sift_process_first_octave(sift, image.data())) {
                    break;
                }
                is_first_octave = false;
            } else {
                if (vl_sift_process_next_octave(sift)) {
                    break;
                }
            }

            // Detect keypoints.
            vl_sift_detect(sift);

            // Octave.
            const float* dog = sift->dog;
            const int octave_width = sift->octave_width;
            const int octave_height = sift->octave_height;

            // Extract detected keypoints.
            const VlSiftKeypoint* vl_keypoints = vl_sift_get_keypoints(sift);
            const int num_keypoints = vl_sift_get_nkeypoints(sift);
            if (num_keypoints == 0) {
                continue;
            }

            // Process keypoints.
            for (int i = 0; i < num_keypoints; ++i) {
                // Extract feature orientations.
                double angles[4];
                int num_orientations;
                if (options_.upright) {
                    num_orientations = 1;
                    angles[0] = 0.0;
                } else {
                    num_orientations = vl_sift_calc_keypoint_orientations(
                        sift, angles, &vl_keypoints[i]);
                }
                for (int o = 0; o < num_orientations; ++o) {
                    keypoints.conservativeResize(num_feats + 1, kdim);
                    scores.conservativeResize(num_feats + 1);
                    descriptors.conservativeResize(num_feats + 1, ddim);
                    // Construct keypoint.
                    keypoints(num_feats, 0) = vl_keypoints[i].x;
                    keypoints(num_feats, 1) = vl_keypoints[i].y;
                    keypoints(num_feats, 2) = vl_keypoints[i].sigma;
                    keypoints(num_feats, 3) = angles[o];
                    // Construct descriptor.
                    vl_sift_calc_keypoint_descriptor(
                        sift,
                        descriptors.row(num_feats).data(),
                        &vl_keypoints[i],
                        angles[o]);
                    // Recover score.
                    scores(num_feats) =
                        dog[vl_keypoints[i].is * octave_width * octave_height +
                            vl_keypoints[i].iy * octave_width +
                            vl_keypoints[i].ix];
                    num_feats++;
                }
            }
        }

        // Delete the SIFT object.
        vl_sift_delete(sift);

        return std::make_tuple(keypoints, scores, descriptors);
    }

    sift_output_t ExtractCPU(Eigen::Ref<const pyimage_t<uint8_t>> image) {
        pyimage_t<float> image_f = image.cast<float>();
        image_f.array() /= 255.0f;
        return ExtractCPU(image_f);
    }

    std::unique_ptr<SiftGPU> sift_gpu;
    SiftExtractionOptions options_;
    bool use_gpu_ = false;
};

// Backward compatibility
sift_output_t extract_sift(const py::array_t<float> image,
                           const int num_octaves,
                           const int octave_resolution,
                           const int first_octave,
                           const float edge_thresh,
                           const float peak_thresh,
                           const bool upright) {
    SiftExtractionOptions options;
    options.num_octaves = num_octaves;
    options.octave_resolution = octave_resolution;
    options.first_octave = first_octave;
    options.edge_threshold = edge_thresh;
    options.peak_threshold = peak_thresh;
    options.upright = upright;
    Sift sift(options, Device::CPU);
    return sift.Extract(image.cast<Eigen::Ref<const pyimage_t<float>>>(),
                        false);
}

void init_sift(py::module& m) {
    m.def("extract_sift",
          &extract_sift,
          py::arg("image"),
          py::arg("num_octaves") = 4,
          py::arg("octave_resolution") = 3,
          py::arg("first_octave") = 0,
          py::arg("edge_thresh") = 10.0,
          py::arg("peak_thresh") = 0.01,
          py::arg("upright") = false,
          "Extract SIFT features.");

    // For backwards consistency
    py::dict sift_options;
    sift_options["peak_threshold"] = 0.01;
    sift_options["first_octave"] = 0;
    sift_options["max_image_size"] = 7000;

    py::class_<Sift>(m, "Sift")
        .def(py::init<SiftExtractionOptions, Device>(),
             py::arg("options") = sift_options,
             py::arg("device") = Device::AUTO)
        .def("extract",
             &Sift::Extract<float>,
             py::arg("image"),
             py::arg("do_normalize") = false)
        .def("extract",
             &Sift::Extract<uint8_t>,
             py::arg("image").noconvert(),
             py::arg("do_normalize") = false)
        .def_property_readonly("options", &Sift::Options)
        .def_property_readonly("device", &Sift::GetDevice);
}
