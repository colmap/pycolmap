#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

#include <Eigen/Core>

extern "C" {
    #include <lib/VLFeat/sift.h>
}

#include <iostream>

#define kdim 4
#define ddim 128

typedef Eigen::Matrix<float, kdim, 1> keypoint_type;
typedef Eigen::Matrix<float, ddim, 1> descriptor_type;

py::tuple extract_sift(
        const py::array_t<float> image,
        const int num_octaves,
        const int octave_resolution,
        const int first_octave,
        const float edge_thresh,
        const float peak_thresh,
        const bool upright
) {
    // Check that input is grayscale.
    assert(image.ndim() == 2);

    // Create a new instance of SIFT detector & descriptor.
    VlSiftFilt *sift = vl_sift_new(image.shape(1), image.shape(0), num_octaves, octave_resolution, first_octave);
    vl_sift_set_edge_thresh(sift, edge_thresh);
    vl_sift_set_peak_thresh(sift, peak_thresh);

    // Recover pointer to image;
    py::buffer_info image_buf = image.request();
    float *image_ptr = (float *)image_buf.ptr;

    // Build image pyramid.
    bool is_first_octave = true;
    std::vector<keypoint_type> keypoints;
    std::vector<descriptor_type> descriptors;
    std::vector<float> scores;
    while (true) {
        if (is_first_octave) {
            if (vl_sift_process_first_octave(sift, image_ptr)) {
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
        const float *dog = sift->dog;
        const int octave_width = sift->octave_width;
        const int octave_height = sift->octave_height;

        // Code for converting DoG to numpy arrays.
        // py::array_t<float> map(
        //     py::detail::any_container<ssize_t>(
        //         {octave_resolution + 2, octave_height, octave_width}
        //     )
        // );
        // py::buffer_info map_buf = map.request();
        // float *map_ptr = (float *)map_buf.ptr;
        // memcpy(map_ptr, dog, (octave_resolution + 2) * octave_height * octave_width * sizeof(float));

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
            if (upright) {
                num_orientations = 1;
                angles[0] = 0.0;
            } else {
                num_orientations = vl_sift_calc_keypoint_orientations(sift, angles, &vl_keypoints[i]);
            }
            for (int o = 0; o < num_orientations; ++o) {
                // Construct keypoint.
                keypoint_type keypoint;
                keypoint << vl_keypoints[i].x, vl_keypoints[i].y, vl_keypoints[i].sigma, angles[o];
                // Construct descriptor.
                descriptor_type descriptor;
                vl_sift_calc_keypoint_descriptor(sift, descriptor.data(), &vl_keypoints[i], angles[o]);
                // Recover score.
                const float score = dog[vl_keypoints[i].is * octave_width * octave_height + vl_keypoints[i].iy * octave_width + vl_keypoints[i].ix];

                // Append to lists.
                keypoints.push_back(keypoint);
                descriptors.push_back(descriptor);
                scores.push_back(score);
            }
        }
    }

    // Delete the SIFT object.
    vl_sift_delete(sift);

    // Allocate the arrays.
    const int num_keypoints = keypoints.size();
    // Keypoints.
    py::array_t<float> pykeypoints(
        py::detail::any_container<ssize_t>(
            {num_keypoints, kdim}
        )
    );
    py::buffer_info pykeypoints_buf = pykeypoints.request();
    float *pykeypoints_ptr = (float *)pykeypoints_buf.ptr;
    // Scores.
    py::array_t<float> pyscores(
        py::detail::any_container<ssize_t>(
            {num_keypoints}
        )
    );
    py::buffer_info pyscores_buf = pyscores.request();
    float *pyscores_ptr = (float *)pyscores_buf.ptr;
    // Descriptors.
    py::array_t<float> pydescriptors(
        py::detail::any_container<ssize_t>(
            {num_keypoints, ddim}
        )
    );
    py::buffer_info pydescriptors_buf = pydescriptors.request();
    float *pydescriptors_ptr = (float *)pydescriptors_buf.ptr;
    // Copy.
    for (int i = 0; i < num_keypoints; ++i) {
        memcpy(pykeypoints_ptr + kdim * i, keypoints[i].data(), kdim * sizeof(float));
        pyscores_ptr[i] = scores[i];
        memcpy(pydescriptors_ptr + ddim * i, descriptors[i].data(), ddim * sizeof(float));
    }

    return py::make_tuple(pykeypoints, pyscores, pydescriptors);
}