// Author: Philipp Lindenberger (Phil26AT)

#include "colmap/scene/image.h"
#include "colmap/geometry/rigid3.h"
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

PYBIND11_MAKE_OPAQUE(std::unordered_map<colmap::image_t, colmap::Image>);

template <typename... Args>
using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;

std::string PrintImage(const colmap::Image& image) {
    std::stringstream ss;
    ss << "<Image 'image_id="
       << (image.ImageId() != kInvalidImageId ? std::to_string(image.ImageId()) : "Invalid")
       << ", camera_id=" << (image.HasCamera() ? std::to_string(image.CameraId()) : "Invalid")
       << ", name=\"" << image.Name() << "\""
       << ", triangulated=" << image.NumPoints3D() << "/" << image.NumPoints2D() << "'>";
    return ss.str();
}

template<typename T>
std::shared_ptr<Image> MakeImage(const std::string& name, const std::vector<T>& points2D,
                                 const Rigid3d& cam_from_world, size_t camera_id,
                                 colmap::image_t image_id) {
    auto image = std::make_shared<Image>();
    image->SetName(name);
    image->SetPoints2D(points2D);
    image->CamFromWorld() = cam_from_world;
    if (camera_id != kInvalidCameraId) {
     image->SetCameraId(camera_id);
    }
    image->SetImageId(image_id);
    return image;
}

void init_image(py::module& m) {
    using ImageMap = std::unordered_map<colmap::image_t, colmap::Image>;
    py::bind_map<ImageMap>(m, "MapImageIdImage").def("__repr__", [](const ImageMap& self) {
        std::string repr = "{";
        bool is_first = true;
        for (auto& pair : self) {
            if (!is_first) {
                repr += ", ";
            }
            is_first = false;
            repr += std::to_string(pair.first) + ": " + PrintImage(pair.second);
        }
        repr += "}";
        return repr;
    });

    py::class_<colmap::Image, std::shared_ptr<colmap::Image>>(m, "Image")
        .def(py::init<>())
        .def(py::init(&MakeImage<Point2D>),
             py::arg("name") = "", py::arg("points2D") = std::vector<Point2D>(),
             py::arg("cam_from_world") = Rigid3d(),
             py::arg("camera_id") = kInvalidCameraId, py::arg("id") = kInvalidImageId)
        .def(py::init(&MakeImage<Eigen::Vector2d>),
             py::arg("name") = "", py::arg("keypoints") = std::vector<Eigen::Vector2d>(),
             py::arg("cam_from_world") = Rigid3d(),
             py::arg("camera_id") = kInvalidCameraId, py::arg("id") = kInvalidImageId)
        .def_property("image_id", &Image::ImageId, &Image::SetImageId,
                      "Unique identifier of image.")
        .def_property(
            "camera_id", &Image::CameraId,
            [](Image& self, const camera_t camera_id) {
                THROW_CHECK_NE(camera_id, kInvalidCameraId);
                self.SetCameraId(kInvalidCameraId);
            },
            "Unique identifier of the camera.")
        .def_property("name", overload_cast_<>()(&Image::Name), &Image::SetName,
                      "Name of the image.")
        .def_property(
            "cam_from_world", overload_cast_<>()(&Image::CamFromWorld),
            [](Image& self, const Rigid3d& cam_from_world) {self.CamFromWorld() = cam_from_world;},
            "The pose of the image, defined as the transformation from world to camera space.")
        .def_property(
            "cam_from_world_prior", overload_cast_<>()(&Image::CamFromWorldPrior),
            [](Image& self, const Rigid3d& cam_from_world) {self.CamFromWorldPrior() = cam_from_world;},
            "The pose prior of the image, e.g. extracted from EXIF tags.")
        .def_property(
            "points2D", overload_cast_<>()(&Image::Points2D),
            [](Image& self, const std::vector<class Point2D>& points2D) {
                THROW_CUSTOM_CHECK(!points2D.empty(), std::invalid_argument);
                self.SetPoints2D(points2D);
            },
            "Array of Points2D (=keypoints).")
        .def(
            "set_point3D_for_point2D",
            [](Image& self, const point2D_t point2D_idx, const point3D_t point3D_id) {
                THROW_CHECK_NE(point3D_id, kInvalidPoint3DId);
                self.SetPoint3DForPoint2D(point2D_idx, point3D_id);
            },
            "Set the point as triangulated, i.e. it is part of a 3D point track.")
        .def("reset_point3D_for_point2D", &Image::ResetPoint3DForPoint2D,
             "Set the point as not triangulated, i.e. it is not part of a 3D point track")
        .def("is_point3D_visible", &Image::IsPoint3DVisible,
             "Check whether an image point has a correspondence to an image point in\n"
             "another image that has a 3D point.")
        .def("has_point3D", &Image::HasPoint3D,
             "Check whether one of the image points is part of the 3D point track.")
        .def("increment_correspondence_has_point3D", &Image::IncrementCorrespondenceHasPoint3D,
             "Indicate that another image has a point that is triangulated and has\n"
             "a correspondence to this image point. Note that this must only be called\n"
             "after calling `SetUp`.")
        .def("decrement_correspondence_has_point3D", &Image::DecrementCorrespondenceHasPoint3D,
             "Indicate that another image has a point that is not triangulated any more\n"
             "and has a correspondence to this image point. This assumes that\n"
             "`IncrementCorrespondenceHasPoint3D` was called for the same image point\n"
             "and correspondence before. Note that this must only be called\n"
             "after calling `SetUp`.")
        .def("projection_center", &Image::ProjectionCenter,
             "Extract the projection center in world space.")
        .def("viewing_direction", &Image::ViewingDirection,
             "Extract the viewing direction of the image.")
        .def(
            "set_up",
            [](Image& self, const class Camera& camera) {
                THROW_CHECK_EQ(self.CameraId(), camera.CameraId());
                self.SetUp(camera);
            },
            "Setup the image and necessary internal data structures before being used in "
            "reconstruction.")
        .def("has_camera", &Image::HasCamera, "Check whether identifier of camera has been set.")
        .def_property("registered", &Image::IsRegistered, &Image::SetRegistered,
                      "Whether image is registered in the reconstruction.")
        .def("num_points2D", &Image::NumPoints2D, "Get the number of image points (keypoints).")
        .def("num_points3D", &Image::NumPoints3D,
             "Get the number of triangulations, i.e. the number of points that\n"
             "are part of a 3D point track.")
        .def_property("num_observations", &Image::NumObservations, &Image::SetNumObservations,
                      "Number of observations, i.e. the number of image points that\n"
                      "have at least one correspondence to another image.")
        .def_property("num_correspondences", &Image::NumCorrespondences,
                      &Image::SetNumCorrespondences,
                      "Number of correspondences for all image points.")
        .def("num_visible_points3D", &Image::NumVisiblePoints3D,
             "Get the number of observations that see a triangulated point, i.e. the\n"
             "number of image points that have at least one correspondence to a\n"
             "triangulated point in another image.")
        .def("point3D_visibility_score", &Image::Point3DVisibilityScore,
             "Get the score of triangulated observations. In contrast to\n"
             "`NumVisiblePoints3D`, this score also captures the distribution\n"
             "of triangulated observations in the image. This is useful to select\n"
             "the next best image in incremental reconstruction, because a more\n"
             "uniform distribution of observations results in more robust registration.")
        .def("get_valid_point2D_ids",
             [](const Image& self) {
                 std::vector<colmap::point2D_t> valid_point2D_ids;

                 for (colmap::point2D_t point2D_idx = 0; point2D_idx < self.NumPoints2D();
                      ++point2D_idx) {
                     if (self.Point2D(point2D_idx).HasPoint3D()) {
                         valid_point2D_ids.push_back(point2D_idx);
                     }
                 }

                 return valid_point2D_ids;
             })
        .def("get_valid_points2D",
             [](const Image& self) {
                 std::vector<colmap::Point2D> valid_points2D;

                 for (colmap::point2D_t point2D_idx = 0; point2D_idx < self.NumPoints2D();
                      ++point2D_idx) {
                     if (self.Point2D(point2D_idx).HasPoint3D()) {
                         valid_points2D.push_back(self.Point2D(point2D_idx));
                     }
                 }

                 return valid_points2D;
             })
        .def(
            "project",
            [](const Image& self, const std::vector<Eigen::Vector3d>& world_coords) {
                std::vector<Eigen::Vector2d> image_points(world_coords.size());
                for (int idx = 0; idx < world_coords.size(); ++idx) {
                    image_points[idx] =
                        (self.CamFromWorld() * world_coords[idx]).hnormalized();
                }
                return image_points;
            },
            "Project list of world points to image (xy).")
        .def(
            "project",
            [](const Image& self, const std::vector<Point3D>& point3Ds) {
                std::vector<Eigen::Vector2d> world_points(point3Ds.size());
                for (int idx = 0; idx < point3Ds.size(); ++idx) {
                    world_points[idx] =
                        (self.CamFromWorld() * point3Ds[idx].XYZ()).hnormalized();
                }
                return world_points;
            },
            "Project list of point3Ds to image (xy).")
        .def(
            "transform_to_image",
            [](const Image& self, const std::vector<Eigen::Vector3d>& world_coords) {
                std::vector<Eigen::Vector3d> image_points(world_coords.size());
                for (int idx = 0; idx < world_coords.size(); ++idx) {
                    image_points[idx] = (self.CamFromWorld() * world_coords[idx]);
                }
                return image_points;
            },
            "Project list of points in world coordinate frame to image coordinates (xyz).")
        .def(
            "transform_to_world",
            [](const Image& self, const std::vector<Eigen::Vector3d>& image_coords) {
                std::vector<Eigen::Vector3d> world_points(image_coords.size());
                for (int idx = 0; idx < image_coords.size(); ++idx) {
                    world_points[idx] = (Inverse(self.CamFromWorld()) * image_coords[idx]);
                }
                return world_points;
            },
            "Project list of image points (with depth) to world coordinate frame.")
        .def("__copy__", [](const Image& self) { return Image(self); })
        .def("__deepcopy__", [](const Image& self, py::dict) { return Image(self); })
        .def("__repr__", [](const Image& self) { return PrintImage(self); })
        .def("summary", [](const Image& self) {
            std::stringstream ss;
            ss << "Image:\n\timage_id = "
               << (self.ImageId() != kInvalidImageId ? std::to_string(self.ImageId()) : "Invalid")
               << "\n\tcamera_id = "
               << (self.HasCamera() ? std::to_string(self.CameraId()) : "Invalid")
               << "\n\tname = " << self.Name() << "\n\ttriangulated = " << self.NumPoints3D() << "/"
               << self.NumPoints2D() << "\n\tcam_from_world = [" << self.CamFromWorld().ToMatrix() << "]";
            return ss.str();
        });
}
