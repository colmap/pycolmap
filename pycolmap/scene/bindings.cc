#include <pybind11/pybind11.h>

#include "pycolmap/scene/camera.cc"
#include "pycolmap/scene/correspondence_graph.cc"
#include "pycolmap/scene/image.cc"
#include "pycolmap/scene/point2D.cc"
#include "pycolmap/scene/point3D.cc"
#include "pycolmap/scene/reconstruction.cc"
#include "pycolmap/scene/track.cc"

namespace py = pybind11;

void BindScene(py::module& m) {
  BindImage(m);
  BindCamera(m);
  BindPoint2D(m);
  BindTrack(m);
  BindPoint3D(m);
  BindCorrespondenceGraph(m);
  BindReconstruction(m);
}
