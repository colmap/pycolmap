// Author: Paul-Edouard Sarlin (skydes)

#include "colmap/exe/sfm.h"
#include "colmap/base/camera_models.h"
#include "colmap/base/reconstruction.h"
#include "colmap/controllers/incremental_mapper.h"
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

#include "pipeline/extract_features.cc"
#include "pipeline/images.cc"
#include "pipeline/match_features.cc"

Reconstruction triangulate_points(Reconstruction reconstruction,
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
    RunPointTriangulatorImpl(reconstruction,
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
    const IncrementalMapperOptions& options,
    const py::object input_path_) {
    std::string database_path = py::str(database_path_).cast<std::string>();
    THROW_CHECK_FILE_EXISTS(database_path);
    std::string image_path = py::str(image_path_).cast<std::string>();
    THROW_CHECK_DIR_EXISTS(image_path);
    std::string input_path = py::str(input_path_).cast<std::string>();
    std::string output_path = py::str(output_path_).cast<std::string>();
    CreateDirIfNotExists(output_path);

    py::gil_scoped_release release;
    ReconstructionManager reconstruction_manager;
    if (input_path != "") {
        THROW_CHECK_DIR_EXISTS(input_path);
        reconstruction_manager.Read(input_path);
    }
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

    PyInterrupt py_interrupt(1.0);  // Check for interrupts every 2 seconds
    mapper.AddCallback(IncrementalMapperController::NEXT_IMAGE_REG_CALLBACK,
                       [&]() {
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
    const int min_num_matches,
    const py::object input_path_) {
    IncrementalMapperOptions options;
    options.num_threads = num_threads;
    options.min_num_matches = min_num_matches;
    return incremental_mapping(
        database_path_, image_path_, output_path_, options, input_path_);
}

void init_sfm(py::module& m) {
    init_images(m);
    init_extract_features(m);
    init_match_features(m);

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
              const IncrementalMapperOptions&,
              const py::object)>(&incremental_mapping),
          py::arg("database_path"),
          py::arg("image_path"),
          py::arg("output_path"),
          py::arg("options") = mapper_options,
          py::arg("input_path") = py::str(""),
          "Triangulate 3D points from known poses");

    m.def("incremental_mapping",
          static_cast<std::map<size_t, Reconstruction> (*)(const py::object,
                                                           const py::object,
                                                           const py::object,
                                                           const int,
                                                           const int,
                                                           const py::object)>(
              &incremental_mapping),
          py::arg("database_path"),
          py::arg("image_path"),
          py::arg("output_path"),
          py::arg("num_threads") = mapper_options.num_threads,
          py::arg("min_num_matches") = mapper_options.min_num_matches,
          py::arg("input_path") = py::str(""),
          "Triangulate 3D points from known poses");
}
