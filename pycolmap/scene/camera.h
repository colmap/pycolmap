// Author: Philipp Lindenberger (Phil26AT)

#include "colmap/scene/camera.h"
#include "colmap/sensor/models.h"
#include "colmap/util/misc.h"
#include "colmap/util/types.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "pycolmap/log_exceptions.h"

using namespace colmap;
using namespace pybind11::literals;
namespace py = pybind11;

using CameraMap = std::unordered_map<camera_t, Camera>;
PYBIND11_MAKE_OPAQUE(CameraMap);

// TODO: cleanup
std::string PrintCamera(const Camera& camera) {
  std::stringstream ss;
  ss << "<Camera 'camera_id="
     << (camera.camera_id != kInvalidCameraId ? std::to_string(camera.camera_id)
                                              : "Invalid")
     << ", model=" << camera.ModelName() << ", width=" << camera.width
     << ", height=" << camera.height << ", num_params=" << camera.params.size()
     << "'>";
  return ss.str();
}

void BindCamera(py::module& m) {
  auto PyCameraModelId = py::enum_<CameraModelId>(m, "CameraModelId")
                             .value("INVALID", CameraModelId::kInvalid);
#define CAMERA_MODEL_CASE(CameraModel) \
  PyCameraModelId.value(CameraModel::model_name.c_str(), CameraModel::model_id);

  CAMERA_MODEL_CASES

#undef CAMERA_MODEL_CASE
  PyCameraModelId.export_values();
  AddStringToEnumConstructor(PyCameraModelId);
  py::implicitly_convertible<int, CameraModelId>();

  py::bind_map<CameraMap>(m, "MapCameraIdCamera");

  py::class_<Camera, std::shared_ptr<Camera>> PyCamera(m, "Camera");
  PyCamera.def(py::init<>())
      .def_static("create",
                  &Camera::CreateFromModelId,
                  "camera_id"_a,
                  "model"_a,
                  "focal_length"_a,
                  "width"_a,
                  "height"_a)
      .def_readwrite(
          "camera_id", &Camera::camera_id, "Unique identifier of the camera.")
      .def_readwrite("model_id", &Camera::model_id, "Camera model ID.")
      .def_property(
          "model_name",
          &Camera::ModelName,
          [](Camera& self, std::string model_name) {
            self.model_id = CameraModelNameToId(model_name);
          },
          "Camera model name (connected to model_id).")
      .def_readwrite("width", &Camera::width, "Width of camera sensor.")
      .def_readwrite("height", &Camera::height, "Height of camera sensor.")
      .def("mean_focal_length", &Camera::MeanFocalLength)
      .def_property(
          "focal_length", &Camera::FocalLength, &Camera::SetFocalLength)
      .def_property(
          "focal_length_x", &Camera::FocalLengthX, &Camera::SetFocalLengthX)
      .def_property(
          "focal_length_y", &Camera::FocalLengthY, &Camera::SetFocalLengthY)
      .def_readwrite("has_prior_focal_length", &Camera::has_prior_focal_length)
      .def_property("principal_point_x",
                    &Camera::PrincipalPointX,
                    &Camera::SetPrincipalPointX)
      .def_property("principal_point_y",
                    &Camera::PrincipalPointY,
                    &Camera::SetPrincipalPointY)
      .def("focal_length_idxs",
           &Camera::FocalLengthIdxs,
           "Indices of focal length parameters in params property.")
      .def("principal_point_idxs",
           &Camera::PrincipalPointIdxs,
           "Indices of principal point parameters in params property.")
      .def("extra_params_idxs",
           &Camera::ExtraParamsIdxs,
           "Indices of extra parameters in params property.")
      .def("calibration_matrix",
           &Camera::CalibrationMatrix,
           "Compute calibration matrix from params.")
      .def("params_info",
           &Camera::ParamsInfo,
           "Get human-readable information about the parameter vector "
           "ordering.")
      .def_property(
          "params",
          [](Camera& self) {
            // Return a view (via a numpy array) instead of a copy.
            return Eigen::Map<Eigen::VectorXd>(self.params.data(),
                                               self.params.size());
          },
          [](Camera& self, const std::vector<double>& params) {
            self.params = params;
          },
          "Camera parameters.")
      .def("params_to_string",
           &Camera::ParamsToString,
           "Concatenate parameters as comma-separated list.")
      .def("set_params_from_string",
           &Camera::SetParamsFromString,
           "Set camera parameters from comma-separated list.")
      .def("verify_params",
           &Camera::VerifyParams,
           "Check whether parameters are valid, i.e. the parameter vector has"
           "\nthe correct dimensions that match the specified camera model.")
      .def("has_bogus_params",
           &Camera::HasBogusParams,
           "Check whether camera has bogus parameters.")
      .def("cam_from_img",
           &Camera::CamFromImg,
           "Project point in image plane to world / infinity.")
      .def(
          "cam_from_img",
          [](const Camera& self, const std::vector<Eigen::Vector2d>& points2D) {
            std::vector<Eigen::Vector2d> world_points2D;
            for (size_t idx = 0; idx < points2D.size(); ++idx) {
              world_points2D.push_back(self.CamFromImg(points2D[idx]));
            }
            return world_points2D;
          },
          "Project list of points in image plane to world / infinity.")
      .def(
          "cam_from_img",
          [](const Camera& self, const std::vector<Point2D>& points2D) {
            std::vector<Eigen::Vector2d> world_points2D;
            for (size_t idx = 0; idx < points2D.size(); ++idx) {
              world_points2D.push_back(self.CamFromImg(points2D[idx].xy));
            }
            return world_points2D;
          },
          "Project list of points in image plane to world / infinity.")
      .def("cam_from_img_threshold",
           &Camera::CamFromImgThreshold,
           "Convert pixel threshold in image plane to world space.")
      .def("img_from_cam",
           &Camera::ImgFromCam,
           "Project point from world / infinity to image plane.")
      .def(
          "img_from_cam",
          [](const Camera& self,
             const std::vector<Eigen::Vector2d>& world_points2D) {
            std::vector<Eigen::Vector2d> image_points2D;
            for (size_t idx = 0; idx < world_points2D.size(); ++idx) {
              image_points2D.push_back(self.ImgFromCam(world_points2D[idx]));
            }
            return image_points2D;
          },
          "Project list of points from world / infinity to image plane.")
      .def(
          "img_from_cam",
          [](const Camera& self, const std::vector<Point2D>& world_points2D) {
            std::vector<Eigen::Vector2d> image_points2D;
            for (size_t idx = 0; idx < world_points2D.size(); ++idx) {
              image_points2D.push_back(self.ImgFromCam(world_points2D[idx].xy));
            }
            return image_points2D;
          },
          "Project list of points from world / infinity to image plane.")
      .def("rescale",
           py::overload_cast<size_t, size_t>(&Camera::Rescale),
           "Rescale camera dimensions to (width_height) and accordingly the "
           "focal length and\n"
           "and the principal point.")
      .def("rescale",
           py::overload_cast<double>(&Camera::Rescale),
           "Rescale camera dimensions by given factor and accordingly the "
           "focal length and\n"
           "and the principal point.")
      .def("__copy__", [](const Camera& self) { return Camera(self); })
      .def("__deepcopy__",
           [](const Camera& self, py::dict) { return Camera(self); })
      .def("__repr__", [](const Camera& self) { return PrintCamera(self); })
      .def("summary", [](const Camera& self) {
        std::stringstream ss;
        ss << "Camera:\n\tcamera_id="
           << (self.camera_id != kInvalidCameraId
                   ? std::to_string(self.camera_id)
                   : "Invalid")
           << "\n\tmodel = " << self.ModelName() << "\n\twidth = " << self.width
           << "\n\theight = " << self.height
           << "\n\tnum_params = " << self.params.size()
           << "\n\tparams_info = " << self.ParamsInfo()
           << "\n\tparams = " << self.ParamsToString();
        return ss.str();
      });
  PyCamera.def(py::init([PyCamera](py::dict dict) {
                 auto self = py::object(PyCamera());
                 for (auto& it : dict) {
                   auto key_str = it.first.cast<std::string>();
                   if ((key_str == "model") || (key_str == "model_name")) {
                     self.attr("model_id") = it.second;  // Implicit conversion.
                   } else {
                     self.attr(it.first) = it.second;
                   }
                 }
                 return self.cast<Camera>();
               }),
               "dict"_a);
  PyCamera.def(py::init([PyCamera](py::kwargs kwargs) {
    py::dict dict = kwargs.cast<py::dict>();
    return PyCamera(dict).cast<Camera>();
  }));

  py::implicitly_convertible<py::dict, Camera>();
}
