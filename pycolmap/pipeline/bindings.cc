#include <pybind11/pybind11.h>

#include "pycolmap/pipeline/extract_features.cc"
#include "pycolmap/pipeline/images.cc"
#include "pycolmap/pipeline/match_features.cc"
#include "pycolmap/pipeline/meshing.cc"
#include "pycolmap/pipeline/mvs.cc"
#include "pycolmap/pipeline/sfm.cc"

namespace py = pybind11;

void BindPipeline(py::module& m) {
  BindImages(m);
  BindExtractFeatures(m);
  BindMatchFeatures(m);
  BindSfM(m);
  BindMVS(m);
  BindMeshing(m);
}
