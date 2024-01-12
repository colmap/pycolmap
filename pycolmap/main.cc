#include "colmap/util/version.h"

#include "pycolmap/estimators/bindings.h"
#include "pycolmap/feature/sift.h"
#include "pycolmap/geometry/bindings.h"
#include "pycolmap/helpers.h"
#include "pycolmap/optim/bindings.h"
#include "pycolmap/pipeline/bindings.h"
#include "pycolmap/scene/bindings.h"
#include "pycolmap/sfm/bindings.h"
#include "pycolmap/utils.h"

#include <glog/logging.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace colmap;

struct Logging {
  enum class Level {
    INFO_ = google::GLOG_INFO,
    WARNING_ = google::GLOG_WARNING,
    ERROR_ = google::GLOG_ERROR,
    FATAL_ = google::GLOG_FATAL,
  };
};  // dummy class

PYBIND11_MODULE(pycolmap, m) {
  m.doc() = "COLMAP plugin";
#ifdef VERSION_INFO
  m.attr("__version__") = py::str(VERSION_INFO);
#else
  m.attr("__version__") = py::str("dev");
#endif
  m.attr("has_cuda") = IsGPU(Device::AUTO);
  m.attr("COLMAP_version") = py::str(GetVersionInfo());
  m.attr("COLMAP_build") = py::str(GetBuildInfo());

  auto PyLogging =
      py::class_<Logging>(m, "logging")
          .def_readwrite_static("minloglevel", &FLAGS_minloglevel)
          .def_readwrite_static("stderrthreshold", &FLAGS_stderrthreshold)
          .def_readwrite_static("log_dir", &FLAGS_log_dir)
          .def_readwrite_static("logtostderr", &FLAGS_logtostderr)
          .def_readwrite_static("alsologtostderr", &FLAGS_alsologtostderr)
          .def_static("info", [](const std::string& msg) { LOG(INFO) << msg; })
          .def_static("warning",
                      [](const std::string& msg) { LOG(WARNING) << msg; })
          .def_static("error",
                      [](const std::string& msg) { LOG(ERROR) << msg; })
          .def_static("fatal",
                      [](const std::string& msg) { LOG(FATAL) << msg; });
  py::enum_<Logging::Level>(PyLogging, "Level")
      .value("INFO", Logging::Level::INFO_)
      .value("WARNING", Logging::Level::WARNING_)
      .value("ERROR", Logging::Level::ERROR_)
      .value("FATAL", Logging::Level::FATAL_)
      .export_values();
  google::InitGoogleLogging("");
  google::InstallFailureSignalHandler();
  FLAGS_logtostderr = true;

  auto PyDevice = py::enum_<Device>(m, "Device")
                      .value("auto", Device::AUTO)
                      .value("cpu", Device::CPU)
                      .value("cuda", Device::CUDA);
  AddStringToEnumConstructor(PyDevice);

  BindGeometry(m);
  BindOptim(m);
  BindScene(m);
  BindEstimators(m);
  BindSfMObjects(m);
  BindSift(m);
  BindPipeline(m);

  py::add_ostream_redirect(m, "ostream");
}
