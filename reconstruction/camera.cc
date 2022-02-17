// Author: Philipp Lindenberger (Phil26AT)

#include "colmap/base/image.h"
#include "colmap/util/misc.h"
#include "colmap/util/types.h"

using namespace colmap;

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;
using namespace pybind11::literals;

#include "log_exceptions.h"

template <typename... Args>
using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;

PYBIND11_MAKE_OPAQUE(EIGEN_STL_UMAP(colmap::camera_t, colmap::Camera));

std::string PrintCamera(const colmap::Camera& camera) {
    std::stringstream ss;
    ss << "<Camera 'camera_id="
       << (camera.CameraId() != kInvalidCameraId ? std::to_string(camera.CameraId()) : "Invalid")
       << ", model=" << camera.ModelName() << ", width=" << camera.Width()
       << ", height=" << camera.Height() << ", num_params=" << camera.NumParams() << "'>";
    return ss.str();
}

void init_camera(py::module& m) {
    using CameraMap = EIGEN_STL_UMAP(colmap::camera_t, colmap::Camera);

    py::bind_map<CameraMap>(m, "MapCameraIdCamera").def("__repr__", [](const CameraMap& self) {
        std::string repr = "{";
        bool is_first = true;
        for (auto& pair : self) {
            if (!is_first) {
                repr += ", ";
            }
            is_first = false;
            repr += std::to_string(pair.first) + ": " + PrintCamera(pair.second);
        }
        repr += "}";
        return repr;
    });

    py::class_<Camera, std::shared_ptr<Camera>>(m, "Camera")
        .def(py::init<>())
        .def(py::init([](const std::string& name, size_t width, size_t height,
                         const std::vector<double>& params, colmap::camera_t camera_id) {
                 std::unique_ptr<Camera> camera = std::unique_ptr<Camera>(new Camera());
                 THROW_CHECK(ExistsCameraModelWithName(name));
                 camera->SetModelIdFromName(name);
                 camera->SetWidth(width);
                 camera->SetHeight(height);
                 camera->SetParams(params);
                 camera->SetCameraId(camera_id);
                 return camera;
             }),
             py::arg("model"), py::arg("width"), py::arg("height"), py::arg("params"),
             py::arg("id") = kInvalidCameraId)
        .def(py::init([](py::dict camera_dict) {
                 std::unique_ptr<Camera> camera = std::unique_ptr<Camera>(new Camera());
                 std::string name = camera_dict["model"].cast<std::string>();
                 THROW_CHECK(ExistsCameraModelWithName(name));
                 camera->SetModelIdFromName(name);
                 camera->SetWidth(camera_dict["width"].cast<size_t>());
                 camera->SetHeight(camera_dict["height"].cast<size_t>());
                 camera->SetParams(camera_dict["params"].cast<std::vector<double>>());
                 return camera;
             }),
             py::arg("camera_dict"))
        .def_property("camera_id", &Camera::CameraId, &Camera::SetCameraId,
                      "Unique identifier of the camera.")
        .def_property(
            "model_id", &Camera::ModelId,
            [](Camera& self, int model_id) {
                THROW_CHECK(ExistsCameraModelWithId(model_id));
                self.SetModelId(model_id);
            },
            "Camera model ID.")
        .def_property(
            "model_name", &Camera::ModelName,
            [](Camera& self, std::string model_name) {
                THROW_CHECK(ExistsCameraModelWithName(model_name));
                self.SetModelIdFromName(model_name);
            },
            "Camera model name (connected to model_id).")
        .def_property("width", &Camera::Width, &Camera::SetWidth, "Width of camera sensor.")
        .def_property("height", &Camera::Height, &Camera::SetHeight, "Height of camera sensor.")
        .def("mean_focal_length", &Camera::MeanFocalLength)
        .def_property("focal_length", &Camera::FocalLength, &Camera::SetFocalLength)
        .def_property("focal_length_x", &Camera::FocalLengthX, &Camera::SetFocalLengthX)
        .def_property("focal_length_y", &Camera::FocalLengthY, &Camera::SetFocalLengthY)
        .def_property("has_prior_focal_length", &Camera::HasPriorFocalLength, &Camera::SetPriorFocalLength)
        .def_property("principal_point_x", &Camera::PrincipalPointX, &Camera::SetPrincipalPointX)
        .def_property("principal_point_y", &Camera::PrincipalPointY, &Camera::SetPrincipalPointY)
        .def("focal_length_idxs", &Camera::FocalLengthIdxs,
             "Indices of focal length parameters in params property.")
        .def("principal_point_idxs", &Camera::PrincipalPointIdxs,
             "Indices of principal point parameters in params property.")
        .def("extra_params_idxs", &Camera::ExtraParamsIdxs,
             "Indices of extra parameters in params property.")
        .def("calibration_matrix", &Camera::CalibrationMatrix,
             "Compute calibration matrix from params.")
        .def("params_info", &Camera::ParamsInfo,
             "Get human-readable information about the parameter vector ordering.")
        .def("num_params", &Camera::NumParams, "Number of raw camera parameters.")
        .def_property(
            "params",
            [](Camera& self) {return Eigen::Map<Eigen::VectorXd>(self.ParamsData(), self.NumParams());},
            &Camera::SetParams,
            "Camera parameters.")
        .def("params_to_string", &Camera::ParamsToString,
             "Concatenate parameters as comma-separated list.")
        .def("set_params_from_string", &Camera::SetParamsFromString,
             "Set camera parameters from comma-separated list.")
        .def("verify_params", &Camera::VerifyParams,
             "Check whether parameters are valid, i.e. the parameter vector has\n"
             "the correct dimensions that match the specified camera model.")
        .def("has_bogus_params", &Camera::HasBogusParams,
             "Check whether camera has bogus parameters.")
        .def(
            "initialize_with_id",
            [](Camera& self, const int model_id, const double focal_length, const size_t width,
               const size_t height) {
                THROW_CHECK(ExistsCameraModelWithId(model_id));
                self.InitializeWithId(model_id, focal_length, width, height);
            },
            "Initialize parameters for given camera model ID and focal length, and set\n"
            "the principal point to be the image center.")
        .def(
            "initialize_with_name",
            [](Camera& self, std::string model_name, const double focal_length, const size_t width,
               const size_t height) {
                THROW_CHECK(ExistsCameraModelWithName(model_name));
                self.InitializeWithName(model_name, focal_length, width, height);
            },
            "Initialize parameters for given camera model name and focal length, and set\n"
            "the principal point to be the image center.")
        .def("image_to_world", &Camera::ImageToWorld,
             "Project point in image plane to world / infinity.")
        .def(
            "image_to_world",
            [](const Camera& self, const std::vector<Eigen::Vector2d>& points2D) {
                std::vector<Eigen::Vector2d> world_points2D;
                for (size_t idx = 0; idx < points2D.size(); ++idx) {
                    world_points2D.push_back(self.ImageToWorld(points2D[idx]));
                }
                return world_points2D;
            },
            "Project list of points in image plane to world / infinity.")
        .def(
            "image_to_world",
            [](const Camera& self, const std::vector<Point2D>& points2D) {
                std::vector<Eigen::Vector2d> world_points2D;
                for (size_t idx = 0; idx < points2D.size(); ++idx) {
                    world_points2D.push_back(self.ImageToWorld(points2D[idx].XY()));
                }
                return world_points2D;
            },
            "Project list of points in image plane to world / infinity.")
        .def("image_to_world_threshold", &Camera::ImageToWorldThreshold,
             "Convert pixel threshold in image plane to world space.")
        .def("world_to_image", &Camera::WorldToImage,
             "Project point from world / infinity to image plane.")
        .def(
            "world_to_image",
            [](const Camera& self, const std::vector<Eigen::Vector2d>& world_points2D) {
                std::vector<Eigen::Vector2d> image_points2D;
                for (size_t idx = 0; idx < world_points2D.size(); ++idx) {
                    image_points2D.push_back(self.WorldToImage(world_points2D[idx]));
                }
                return image_points2D;
            },
            "Project list of points from world / infinity to image plane.")
        .def(
            "world_to_image",
            [](const Camera& self, const std::vector<Point2D>& world_points2D) {
                std::vector<Eigen::Vector2d> image_points2D;
                for (size_t idx = 0; idx < world_points2D.size(); ++idx) {
                    image_points2D.push_back(self.WorldToImage(world_points2D[idx].XY()));
                }
                return image_points2D;
            },
            "Project list of points from world / infinity to image plane.")
        .def("rescale", overload_cast_<size_t, size_t>()(&Camera::Rescale),
             "Rescale camera dimensions to (width_height) and accordingly the focal length and\n"
             "and the principal point.")
        .def("rescale", overload_cast_<double>()(&Camera::Rescale),
             "Rescale camera dimensions by given factor and accordingly the focal length and\n"
             "and the principal point.")
        .def("__copy__", [](const Camera& self) { return Camera(self); })
        .def("__deepcopy__", [](const Camera& self, py::dict) { return Camera(self); })
        .def("__repr__", [](const Camera& self) { return PrintCamera(self); })
        .def("summary", [](const Camera& self) {
            std::stringstream ss;
            ss << "Camera:\n\tcamera_id="
               << (self.CameraId() != kInvalidCameraId ? std::to_string(self.CameraId())
                                                       : "Invalid")
               << "\n\tmodel = " << self.ModelName() << "\n\twidth = " << self.Width()
               << "\n\theight = " << self.Height() << "\n\tnum_params = " << self.NumParams()
               << "\n\tparams_info = " << self.ParamsInfo()
               << "\n\tparams = " << self.ParamsToString();
            return ss.str();
        });
}
