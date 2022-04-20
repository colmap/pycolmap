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

#include "colmap/base/reconstruction.h"
#include "colmap/mvs/fusion.h"
#include "colmap/mvs/meshing.h"
#include "colmap/mvs/patch_match.h"
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

void patch_match_stereo(py::object workspace_path_,
                        std::string workspace_format,
                        std::string pmvs_option_name,
                        mvs::PatchMatchOptions options,
                        std::string config_path,
                        bool verbose) {
#ifndef CUDA_ENABLED
    THROW_EXCEPTION(
        std::runtime_error,
        "ERROR: Dense stereo reconstruction requires CUDA, which is not "
        "available on your system.");
    return;
#endif  // CUDA_ENABLED
    std::string workspace_path = py::str(workspace_path_).cast<std::string>();
    THROW_CHECK_DIR_EXISTS(workspace_path);

    StringToLower(&workspace_format);
    THROW_CUSTOM_CHECK_MSG(
        (workspace_format == "colmap" || workspace_format == "pmvs"),
        std::invalid_argument,
        "Invalid `workspace_format` - supported values are "
        "'COLMAP' or 'PMVS'.")

    std::stringstream oss;
    std::streambuf* oldcout = nullptr;
    if (!verbose) {
        oldcout = std::cout.rdbuf(oss.rdbuf());
    }

    mvs::PatchMatchController controller(options,
                                         workspace_path,
                                         workspace_format,
                                         pmvs_option_name,
                                         config_path);

    controller.Start();
    PyWait(&controller);
    // controller.Wait();

    if (!verbose) {
        std::cout.rdbuf(oldcout);
    }
}

colmap::Reconstruction stereo_fusion(py::object output_path_,
                                     py::object workspace_path_,
                                     std::string workspace_format,
                                     std::string pmvs_option_name,
                                     std::string input_type,
                                     mvs::StereoFusionOptions options,
                                     bool verbose) {
    std::string workspace_path = py::str(workspace_path_).cast<std::string>();
    THROW_CHECK_DIR_EXISTS(workspace_path);

    std::string output_path = py::str(output_path_).cast<std::string>();

    StringToLower(&workspace_format);
    THROW_CUSTOM_CHECK_MSG(
        (workspace_format == "colmap" || workspace_format == "pmvs"),
        std::invalid_argument,
        "Invalid `workspace_format` - supported values are "
        "'COLMAP' or 'PMVS'.");

    StringToLower(&input_type);
    THROW_CUSTOM_CHECK_MSG(
        (input_type == "photometric" || input_type == "geometric"),
        std::invalid_argument,
        "Invalid input type - supported values are 'photometric' and "
        "'geometric'.");

    mvs::StereoFusion fuser(options,
                            workspace_path,
                            workspace_format,
                            pmvs_option_name,
                            input_type);

    std::stringstream oss;
    std::streambuf* oldcout = nullptr;
    if (!verbose) {
        oldcout = std::cout.rdbuf(oss.rdbuf());
    }
    py::gil_scoped_release release;
    fuser.Start();
    PyWait(&fuser);

    if (!verbose) {
        std::cout.rdbuf(oldcout);
    }

    Reconstruction reconstruction;

    // read data from sparse reconstruction
    if (workspace_format == "colmap") {
        reconstruction.Read(JoinPaths(workspace_path, "sparse"));
    }

    // overwrite sparse point cloud with dense point cloud from fuser
    reconstruction.ImportPLY(fuser.GetFusedPoints());

    if (ExistsDir(output_path)) {
        reconstruction.WriteBinary(output_path);
    } else {
        THROW_CHECK_HAS_FILE_EXTENSION(output_path, ".ply")
        THROW_CHECK_FILE_OPEN(output_path);
        WriteBinaryPlyPoints(output_path, fuser.GetFusedPoints());
        mvs::WritePointsVisibility(output_path + ".vis",
                                   fuser.GetFusedPointsVisibility());
    }

    return reconstruction;
}

void init_mvs(py::module& m) {
    using PMOpts = mvs::PatchMatchOptions;
    auto PyPatchMatchOptions =
        py::class_<PMOpts>(m, "PatchMatchOptions")
            .def(py::init<>())
            .def_readwrite("max_image_size",
                           &PMOpts::max_image_size,
                           "Maximum image size in either dimension.")
            .def_readwrite(
                "gpu_index",
                &PMOpts::gpu_index,
                "Index of the GPU used for patch match. For multi-GPU usage, "
                "you should separate multiple GPU indices by comma, e.g., "
                "\"0,1,2,3\".")
            .def_readwrite("depth_min", &PMOpts::depth_min)
            .def_readwrite("depth_max", &PMOpts::depth_max)
            .def_readwrite(
                "window_radius",
                &PMOpts::window_radius,
                "Half window size to compute NCC photo-consistency cost.")
            .def_readwrite("window_step",
                           &PMOpts::window_step,
                           "Number of pixels to skip when computing NCC.")
            .def_readwrite("sigma_spatial",
                           &PMOpts::sigma_spatial,
                           "Spatial sigma for bilaterally weighted NCC.")
            .def_readwrite("sigma_color",
                           &PMOpts::sigma_color,
                           "Color sigma for bilaterally weighted NCC.")
            .def_readwrite(
                "num_samples",
                &PMOpts::num_samples,
                "Number of random samples to draw in Monte Carlo sampling.")
            .def_readwrite("ncc_sigma",
                           &PMOpts::ncc_sigma,
                           "Spread of the NCC likelihood function.")
            .def_readwrite("min_triangulation_angle",
                           &PMOpts::min_triangulation_angle,
                           "Minimum triangulation angle in degrees.")
            .def_readwrite("incident_angle_sigma",
                           &PMOpts::incident_angle_sigma,
                           "Spread of the incident angle likelihood function.")
            .def_readwrite("num_iterations",
                           &PMOpts::num_iterations,
                           "Number of coordinate descent iterations.")
            .def_readwrite("geom_consistency",
                           &PMOpts::geom_consistency,
                           "Whether to add a regularized geometric consistency "
                           "term to the cost function. If true, the "
                           "`depth_maps` and `normal_maps` must not be null.")
            .def_readwrite("geom_consistency_regularizer",
                           &PMOpts::geom_consistency_regularizer,
                           "The relative weight of the geometric consistency "
                           "term w.r.t. to the photo-consistency term.")
            .def_readwrite("geom_consistency_max_cost",
                           &PMOpts::geom_consistency_max_cost,
                           "Maximum geometric consistency cost in terms of the "
                           "forward-backward reprojection error in pixels.")
            .def_readwrite(
                "filter", &PMOpts::filter, "Whether to enable filtering.")
            .def_readwrite(
                "filter_min_ncc",
                &PMOpts::filter_min_ncc,
                "Minimum NCC coefficient for pixel to be photo-consistent.")
            .def_readwrite("filter_min_triangulation_angle",
                           &PMOpts::filter_min_triangulation_angle,
                           "Minimum triangulation angle to be stable.")
            .def_readwrite(
                "filter_min_num_consistent",
                &PMOpts::filter_min_num_consistent,
                "Minimum number of source images have to be consistent "
                "for pixel not to be filtered.")
            .def_readwrite(
                "filter_geom_consistency_max_cost",
                &PMOpts::filter_geom_consistency_max_cost,
                "Maximum forward-backward reprojection error for pixel "
                "to be geometrically consistent.")
            .def_readwrite("cache_size",
                           &PMOpts::cache_size,
                           "Cache size in gigabytes for patch match.")
            .def_readwrite(
                "allow_missing_files",
                &PMOpts::allow_missing_files,
                "Whether to tolerate missing images/maps in the problem setup")
            .def_readwrite("write_consistency_graph",
                           &PMOpts::write_consistency_graph,
                           "Whether to write the consistency graph.");
    make_dataclass(PyPatchMatchOptions);
    auto patch_match_options = PyPatchMatchOptions().cast<PMOpts>();

    using SFOpts = mvs::StereoFusionOptions;
    auto PyStereoFusionOptions =
        py::class_<SFOpts>(m, "StereoFusionOptions")
            .def(py::init<>())
            .def_readwrite("mask_path",
                           &SFOpts::mask_path,
                           "Path for PNG masks. Same format expected as "
                           "ImageReaderOptions.")
            .def_readwrite("num_threads",
                           &SFOpts::num_threads,
                           "The number of threads to use during fusion.")
            .def_readwrite("max_image_size",
                           &SFOpts::max_image_size,
                           "Maximum image size in either dimension.")
            .def_readwrite("min_num_pixels",
                           &SFOpts::min_num_pixels,
                           "Minimum number of fused pixels to produce a point.")
            .def_readwrite(
                "max_num_pixels",
                &SFOpts::max_num_pixels,
                "Maximum number of pixels to fuse into a single point.")
            .def_readwrite("max_traversal_depth",
                           &SFOpts::max_traversal_depth,
                           "Maximum depth in consistency graph traversal.")
            .def_readwrite("max_reproj_error",
                           &SFOpts::max_reproj_error,
                           "Maximum relative difference between measured and "
                           "projected pixel.")
            .def_readwrite("max_depth_error",
                           &SFOpts::max_depth_error,
                           "Maximum relative difference between measured and "
                           "projected depth.")
            .def_readwrite("max_normal_error",
                           &SFOpts::max_normal_error,
                           "Maximum angular difference in degrees of normals "
                           "of pixels to be fused.")
            .def_readwrite("check_num_images",
                           &SFOpts::check_num_images,
                           "Number of overlapping images to transitively check "
                           "for fusing points.")
            .def_readwrite(
                "use_cache",
                &SFOpts::use_cache,
                "Flag indicating whether to use LRU cache or pre-load all data")
            .def_readwrite("cache_size",
                           &SFOpts::cache_size,
                           "Cache size in gigabytes for fusion.")
            .def_readwrite("bounding_box",
                           &SFOpts::bounding_box,
                           "Bounding box Tuple[min, max]");

    make_dataclass(PyStereoFusionOptions);
    auto stereo_fusion_options = PyStereoFusionOptions().cast<SFOpts>();

    m.def("patch_match_stereo",
          &patch_match_stereo,
          py::arg("workspace_path"),
          py::arg("workspace_format") = "COLMAP",
          py::arg("pmvs_option_name") = "option-all",
          py::arg("options") = patch_match_options,
          py::arg("config_path") = "",
          py::arg("verbose") = true,
          "Runs Patch-Match-Stereo (requires CUDA)");

    m.def("stereo_fusion",
          &stereo_fusion,
          py::arg("output_path"),
          py::arg("workspace_path"),
          py::arg("workspace_format") = "COLMAP",
          py::arg("pmvs_option_name") = "option-all",
          py::arg("input_type") = "geometric",
          py::arg("options") = stereo_fusion_options,
          py::arg("verbose") = true,
          "Stereo Fusion");
}
