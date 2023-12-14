// Author: Paul-Edouard Sarlin (skydes)
#include "colmap/controllers/bundle_adjustment.h"
#include "colmap/controllers/incremental_mapper.h"
#include "colmap/exe/sfm.h"
#include "colmap/scene/reconstruction.h"
#include "colmap/sensor/models.h"
#include "colmap/util/misc.h"

#include "pycolmap/helpers.h"
#include "pycolmap/log_exceptions.h"

#include <memory>

#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

using namespace colmap;
using namespace pybind11::literals;
namespace py = pybind11;

std::shared_ptr<Reconstruction> TriangulatePoints(
    const std::shared_ptr<Reconstruction> reconstruction,
    const py::object database_path_,
    const py::object image_path_,
    const py::object output_path_,
    const bool clear_points,
    const IncrementalMapperOptions& options,
    const bool refine_intrinsics) {
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
                           clear_points,
                           refine_intrinsics);
  return reconstruction;
}

std::map<size_t, std::shared_ptr<Reconstruction>> IncrementalMapping(
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
  auto reconstruction_manager = std::make_shared<ReconstructionManager>();
  if (input_path != "") {
    THROW_CHECK_DIR_EXISTS(input_path);
    reconstruction_manager->Read(input_path);
  }
  auto options_ = std::make_shared<IncrementalMapperOptions>(options);
  IncrementalMapperController mapper(
      options_, image_path, database_path, reconstruction_manager);

  // In case a new reconstruction is started, write results of individual sub-
  // models to as their reconstruction finishes instead of writing all results
  // after all reconstructions finished.
  size_t prev_num_reconstructions = 0;
  std::map<size_t, std::shared_ptr<Reconstruction>> reconstructions;
  mapper.AddCallback(
      IncrementalMapperController::LAST_IMAGE_REG_CALLBACK, [&]() {
        // If the number of reconstructions has not changed, the last model
        // was discarded for some reason.
        if (reconstruction_manager->Size() > prev_num_reconstructions) {
          const std::string reconstruction_path =
              JoinPaths(output_path, std::to_string(prev_num_reconstructions));
          const auto& reconstruction =
              reconstruction_manager->Get(prev_num_reconstructions);
          CreateDirIfNotExists(reconstruction_path);
          reconstruction->Write(reconstruction_path);
          reconstructions[prev_num_reconstructions] = reconstruction;
          prev_num_reconstructions = reconstruction_manager->Size();
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

void BundleAdjustment(std::shared_ptr<Reconstruction> reconstruction,
                      const BundleAdjustmentOptions& options) {
  py::gil_scoped_release release;
  OptionManager option_manager;
  *option_manager.bundle_adjustment = options;
  BundleAdjustmentController controller(option_manager, reconstruction);
  controller.Start();
  PyWait(&controller);
}

void BindSfM(py::module& m) {
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
          .def_readwrite("ba_global_images_freq", &Opts::ba_global_images_freq)
          .def_readwrite("ba_global_points_freq", &Opts::ba_global_points_freq)
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

  using BAOpts = BundleAdjustmentOptions;
  auto PyBALossFunctionType =
      py::enum_<BAOpts::LossFunctionType>(m, "LossFunctionType")
          .value("TRIVIAL", BAOpts::LossFunctionType::TRIVIAL)
          .value("SOFT_L1", BAOpts::LossFunctionType::SOFT_L1)
          .value("CAUCHY", BAOpts::LossFunctionType::CAUCHY);
  AddStringToEnumConstructor(PyBALossFunctionType);
  using CSOpts = ceres::Solver::Options;
  auto PyCeresSolverOptions =
      py::class_<CSOpts>(
          m,
          "CeresSolverOptions",
          // If ceres::Solver::Options is registered by pycolmap AND a
          // downstream library, importing the downstream library results in
          // error:
          //   ImportError: generic_type: type "CeresSolverOptions" is already
          //   registered!
          // Adding a `py::module_local()` fixes this.
          // https://github.com/pybind/pybind11/issues/439#issuecomment-1338251822
          py::module_local())
          .def(py::init<>())
          .def_readwrite("function_tolerance", &CSOpts::function_tolerance)
          .def_readwrite("gradient_tolerance", &CSOpts::gradient_tolerance)
          .def_readwrite("parameter_tolerance", &CSOpts::parameter_tolerance)
          .def_readwrite("minimizer_progress_to_stdout",
                         &CSOpts::minimizer_progress_to_stdout)
          .def_readwrite("minimizer_progress_to_stdout",
                         &CSOpts::minimizer_progress_to_stdout)
          .def_readwrite("max_num_iterations", &CSOpts::max_num_iterations)
          .def_readwrite("max_linear_solver_iterations",
                         &CSOpts::max_linear_solver_iterations)
          .def_readwrite("max_num_consecutive_invalid_steps",
                         &CSOpts::max_num_consecutive_invalid_steps)
          .def_readwrite("max_consecutive_nonmonotonic_steps",
                         &CSOpts::max_consecutive_nonmonotonic_steps)
          .def_readwrite("num_threads", &CSOpts::num_threads);
  make_dataclass(PyCeresSolverOptions);
  auto PyBundleAdjustmentOptions =
      py::class_<BAOpts>(m, "BundleAdjustmentOptions")
          .def(py::init<>())
          .def_readwrite("loss_function_type",
                         &BAOpts::loss_function_type,
                         "Loss function types: Trivial (non-robust) and Cauchy "
                         "(robust) loss.")
          .def_readwrite("loss_function_scale",
                         &BAOpts::loss_function_scale,
                         "Scaling factor determines residual at which "
                         "robustification takes place.")
          .def_readwrite("refine_focal_length",
                         &BAOpts::refine_focal_length,
                         "Whether to refine the focal length parameter group.")
          .def_readwrite(
              "refine_principal_point",
              &BAOpts::refine_principal_point,
              "Whether to refine the principal point parameter group.")
          .def_readwrite("refine_extra_params",
                         &BAOpts::refine_extra_params,
                         "Whether to refine the extra parameter group.")
          .def_readwrite("refine_extrinsics",
                         &BAOpts::refine_extrinsics,
                         "Whether to refine the extrinsic parameter group.")
          .def_readwrite("print_summary",
                         &BAOpts::print_summary,
                         "Whether to print a final summary.")
          .def_readwrite("min_num_residuals_for_multi_threading",
                         &BAOpts::min_num_residuals_for_multi_threading,
                         "Minimum number of residuals to enable "
                         "multi-threading. Note that "
                         "single-threaded is typically better for small bundle "
                         "adjustment problems "
                         "due to the overhead of threading. ")
          .def_readwrite("solver_options",
                         &BAOpts::solver_options,
                         "Ceres-Solver options.");
  make_dataclass(PyBundleAdjustmentOptions);
  auto ba_options = PyBundleAdjustmentOptions().cast<BAOpts>();

  m.def("triangulate_points",
        &TriangulatePoints,
        "reconstruction"_a,
        "database_path"_a,
        "image_path"_a,
        "output_path"_a,
        "clear_points"_a = true,
        "options"_a = mapper_options,
        "refine_intrinsics"_a = false,
        "Triangulate 3D points from known camera poses");

  m.def("incremental_mapping",
        &IncrementalMapping,
        "database_path"_a,
        "image_path"_a,
        "output_path"_a,
        "options"_a = mapper_options,
        "input_path"_a = py::str(""),
        "Triangulate 3D points from known poses");

  m.def("bundle_adjustment",
        &BundleAdjustment,
        "reconstruction"_a,
        "options"_a = ba_options);
}
