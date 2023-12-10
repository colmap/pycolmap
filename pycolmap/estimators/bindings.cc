#include <pybind11/pybind11.h>

#include "pycolmap/estimators/absolute_pose.cc"
#include "pycolmap/estimators/alignment.cc"
#include "pycolmap/estimators/essential_matrix.cc"
#include "pycolmap/estimators/fundamental_matrix.cc"
#include "pycolmap/estimators/generalized_absolute_pose.cc"
#include "pycolmap/estimators/homography_matrix.cc"
#include "pycolmap/estimators/triangulation.cc"
#include "pycolmap/estimators/two_view_geometry.cc"

namespace py = pybind11;

void BindEstimators(py::module& m) {
  BindAbsolutePoseEstimator(m);
  BindAlignmentEstimator(m);
  BindEssentialMatrixEstimator(m);
  BindFundamentalMatrixEstimator(m);
  BindGeneralizedAbsolutePoseEstimator(m);
  BindHomographyMatrixEstimator(m);
  BindTriangulationEstimator(m);
  BindTwoViewGeometryEstimator(m);
}
