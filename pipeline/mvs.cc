#include "colmap/base/reconstruction.h"
#include "colmap/base/image_reader.h"
#include "colmap/base/camera_models.h"
#include "colmap/util/misc.h"
#include "colmap/feature/sift.h"
#include "colmap/feature/extraction.h"
#include "colmap/feature/matching.h"
#include "colmap/controllers/incremental_mapper.h"
#include "colmap/exe/feature.h"
#include "colmap/exe/sfm.h"

#include "colmap/base/reconstruction.h"
#include "colmap/mvs/fusion.h"
#include "colmap/mvs/meshing.h"
#include "colmap/mvs/patch_match.h"
#include "colmap/util/misc.h"

using namespace colmap;

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/iostream.h>

namespace py = pybind11;
using namespace pybind11::literals;

#include "log_exceptions.h"
#include "helpers.h"


// bsub -W 03:59 -n 128 -R "rusage[mem=3000, ngpus_excl_p=1]" python -m pixsfm.eval.TT.run --scenes Ignatius --config mvs --overwrite
void patch_match_stereo(std::string workspace_path, std::string workspace_format,
                        bool geom_consistency, bool verbose) {
  #ifndef CUDA_ENABLED
    THROW_EXCEPTION(std::runtime_error,
                    "ERROR: Dense stereo reconstruction requires CUDA, which is not "
                    "available on your system.")
    return;
  #endif   // CUDA_ENABLED
  std::string pmvs_option_name = "option-all";
  std::string config_path = "";

  StringToLower(&workspace_format);
  if (workspace_format != "colmap" && workspace_format != "pmvs") {
    std::cout << "ERROR: Invalid `workspace_format` - supported values are "
                 "'COLMAP' or 'PMVS'."
              << std::endl;
    return;
  }

  mvs::PatchMatchOptions options;
  options.geom_consistency = geom_consistency;

  std::stringstream oss;
  std::streambuf* oldcerr = nullptr;
  std::streambuf* oldcout = nullptr;
  if (!verbose) {
    // oldcerr = std::cerr.rdbuf( oss.rdbuf() );
    oldcout = std::cout.rdbuf( oss.rdbuf() );
  }

  mvs::PatchMatchController controller(options,
                                       workspace_path, workspace_format,
                                       pmvs_option_name, config_path);

  controller.Start();
  controller.Wait();

  if (!verbose) {
    // std::cerr.rdbuf(oldcerr);
    std::cout.rdbuf(oldcout);
  }
}

int stereo_fusion(std::string workspace_path, std::string output_path,
                  std::string workspace_format, std::string input_type) {
  std::string pmvs_option_name = "option-all";
  std::string output_type = "PLY";
  std::string bbox_path = "";

  StringToLower(&workspace_format);
  if (workspace_format != "colmap" && workspace_format != "pmvs") {
    std::cout << "ERROR: Invalid `workspace_format` - supported values are "
                 "'COLMAP' or 'PMVS'."
              << std::endl;
    return EXIT_FAILURE;
  }

  StringToLower(&input_type);
  if (input_type != "photometric" && input_type != "geometric") {
    std::cout << "ERROR: Invalid input type - supported values are "
                 "'photometric' and 'geometric'."
              << std::endl;
    return EXIT_FAILURE;
  }

  mvs::StereoFusionOptions options;

  if (!bbox_path.empty()) {
    std::ifstream file(bbox_path);
    if (file.is_open()) {
      auto& min_bound = options.bounding_box.first;
      auto& max_bound = options.bounding_box.second;
      file >> min_bound(0) >> min_bound(1) >> min_bound(2);
      file >> max_bound(0) >> max_bound(1) >> max_bound(2);
    } else {
      std::cout << "WARN: Invalid bounds path: \"" << bbox_path
                << "\" - continuing without bounds check" << std::endl;
    }
  }

  mvs::StereoFusion fuser(options, workspace_path,
                          workspace_format, pmvs_option_name, input_type);

  fuser.Start();
  fuser.Wait();

  Reconstruction reconstruction;

  // read data from sparse reconstruction
  if (workspace_format == "colmap") {
    reconstruction.Read(JoinPaths(workspace_path, "sparse"));
  }

  // overwrite sparse point cloud with dense point cloud from fuser
  reconstruction.ImportPLY(fuser.GetFusedPoints());

  std::cout << "Writing output: " << output_path << std::endl;

  // write output
  StringToLower(&output_type);
  if (output_type == "bin") {
    reconstruction.WriteBinary(output_path);
  } else if (output_type == "txt") {
    reconstruction.WriteText(output_path);
  } else if (output_type == "ply") {
    WriteBinaryPlyPoints(output_path, fuser.GetFusedPoints());
    mvs::WritePointsVisibility(output_path + ".vis",
                               fuser.GetFusedPointsVisibility());
  } else {
    std::cerr << "ERROR: Invalid `output_type`" << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}


void init_mvs(py::module& m) {
  m.def("patch_match_stereo", &patch_match_stereo);
  m.def("stereo_fusion", &stereo_fusion);
}


