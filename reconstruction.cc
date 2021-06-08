#include <colmap/base/reconstruction.h>
#include <colmap/util/ply.h>
#include <colmap/base/projection.h>
#include <colmap/util/misc.h>
#include "base/camera_models.h"
#include "log_exceptions.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>

namespace py = pybind11;

template<typename... Args>
      using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;

PYBIND11_MAKE_OPAQUE(EIGEN_STL_UMAP(colmap::point3D_t, colmap::Point3D));
PYBIND11_MAKE_OPAQUE(EIGEN_STL_UMAP(colmap::image_t, colmap::Image));
PYBIND11_MAKE_OPAQUE(EIGEN_STL_UMAP(colmap::camera_t, colmap::Camera));
PYBIND11_MAKE_OPAQUE(std::vector<class Point2D>);

using namespace colmap;

bool ExistsReconstructionText(const std::string& path) {
    return (ExistsFile(JoinPaths(path, "cameras.txt")) &&
    ExistsFile(JoinPaths(path, "images.txt")) &&
    ExistsFile(JoinPaths(path, "points3D.txt")));
}

bool ExistsReconstructionBinary(const std::string& path) {
    return (ExistsFile(JoinPaths(path, "cameras.bin")) &&
    ExistsFile(JoinPaths(path, "images.bin")) &&
    ExistsFile(JoinPaths(path, "points3D.bin")));
}

bool ExistsReconstruction(const std::string& path) {
    return (ExistsReconstructionText(path) || ExistsReconstructionBinary(path));
}

//Reconstruction Bindings
void init_reconstruction(py::module &m) {
    // STL Containers
    py::bind_map<EIGEN_STL_UMAP(colmap::point3D_t, colmap::Point3D)>(m, "MapPoint3DIdPoint3D");
    py::bind_map<EIGEN_STL_UMAP(colmap::image_t, colmap::Image)>(m, "MapImageIdImage");
    py::bind_map<EIGEN_STL_UMAP(colmap::camera_t, colmap::Camera)>(m, "MapCameraIdCamera");
    py::bind_vector<std::vector<class Point2D>>(m,"ListPoint2D");

    py::class_<TrackElement, std::shared_ptr<TrackElement>>(m, "TrackElement")
        .def(py::init<>())
        .def(py::init<image_t, point2D_t>())
        .def_readwrite("image_id", &TrackElement::image_id)
        .def_readwrite("point2D_idx", &TrackElement::point2D_idx)
        .def("__copy__",  [](const TrackElement &self) {
            return TrackElement(self);
        })
        .def("__deepcopy__",  [](const TrackElement &self, py::dict) {
            return TrackElement(self);
        })
        .def("__repr__", [](const TrackElement &self) {
            return "<TrackElement 'image_id=" 
                + std::to_string(self.image_id) 
                + ",point2D_idx=" 
                + std::to_string(self.point2D_idx)
                +"'>";
        });

    py::class_<Track, std::shared_ptr<Track>>(m, "Track")
        .def(py::init<>())
        .def(py::init([](const std::vector<TrackElement>& elements){
            std::unique_ptr<Track> track = std::unique_ptr<Track>(new Track());
            track->AddElements(elements);
            return track;
        }))
        .def("length", &Track::Length, "Track Length.")
        .def("add_element", overload_cast_<image_t, point2D_t>()(&Track::AddElement), 
                "Add observation (image_id, point2D_idx) to track.")
        .def("delete_element", overload_cast_<image_t, point2D_t>()(&Track::DeleteElement), 
                "Delete observation (image_id, point2D_idx) from track.")
        //.def("append", overload_cast_<TrackElement>()(&Track::AddElement))
        .def("add_elements", &Track::AddElements, "Add TrackElement list.")
        .def("remove", [](Track &self, const size_t idx){
                THROW_CHECK_LT(idx, self.Elements().size());
        }, "Remove TrackElement at index.")
        .def_property("elements", &Track::Elements, &Track::SetElements)
        .def("__copy__",  [](const Track &self) {
            return Track(self);
        })
        .def("__deepcopy__",  [](const Track &self, py::dict) {
            return Track(self);
        })
        .def("__repr__", [](const Track &self) {
            return "<Track 'length=" + std::to_string(self.Length()) + "'>";
        });

    py::class_<colmap::Point2D, std::shared_ptr<colmap::Point2D>>(m, "Point2D")
        .def(py::init<>())
        .def(py::init([](const Eigen::Vector2d& xy, size_t point3D_id){
            std::unique_ptr<Point2D> point2D = std::unique_ptr<Point2D>(new Point2D());
            point2D->SetXY(xy);
            point2D->SetPoint3DId(point3D_id);
            return point2D;
        }), py::arg("xy"), py::arg("point3D_id") = kInvalidPoint3DId)
        .def_property("xy", overload_cast_<>()(&Point2D::XY), &Point2D::SetXY)
        .def_property("x", &Point2D::X, [](Point2D &self, double x){
            self.SetXY(Eigen::Vector2d(x, self.Y()));
        })
        .def_property("y", &Point2D::Y, [](Point2D &self, double y){
            self.SetXY(Eigen::Vector2d(self.X(), y));
        })
        .def_property("point3D_id", &Point2D::Point3DId, &Point2D::SetPoint3DId)
        .def("has_point3D", &Point2D::HasPoint3D)
        .def("__copy__",  [](const Point2D &self) {
            return Point2D(self);
        })
        .def("__deepcopy__",  [](const Point2D &self, py::dict) {
            return Point2D(self);
        })
        .def("__repr__", [](const Point2D &self) {
            std::stringstream ss;
            ss<<"<Point2D 'xy=["
                <<self.XY().transpose()
                <<"], point3D_id=" 
                <<(self.HasPoint3D() ? std::to_string(self.Point3DId()) : "Invalid")
                <<"'>";
            return ss.str();
        });
    
    py::class_<colmap::Point3D, std::shared_ptr<colmap::Point3D>>(m, "Point3D")
        .def(py::init<>())
        .def(py::init([](const Eigen::Vector3d& xyz, const Track& track){
            std::unique_ptr<Point3D> point3D = std::unique_ptr<Point3D>(new Point3D());
            point3D->SetXYZ(xyz);
            point3D->SetTrack(track);
            return point3D;
        }), py::arg("xyz"), py::arg("track") = Track())
        .def_property("xyz", overload_cast_<>()(&Point3D::XYZ), &Point3D::SetXYZ)
        .def_property("x", &Point3D::X, [](Point3D &self, double x){
            self.SetXYZ(Eigen::Vector3d(x, self.Y(), self.Z()));
        })
        .def_property("y", &Point3D::Y, [](Point3D &self, double y){
            self.SetXYZ(Eigen::Vector3d(self.X(), y, self.Z()));
        })
        .def_property("z", &Point3D::Z, [](Point3D &self, double z){
            self.SetXYZ(Eigen::Vector3d(self.X(), self.Y(), z));
        })
        .def_property("color", overload_cast_<>()(&Point3D::Color), &Point3D::SetColor)
        .def_property("error", &Point3D::Error, &Point3D::SetError)
        .def_property("track", overload_cast_<>()(&Point3D::Track), &Point3D::SetTrack, 
                                py::return_value_policy::reference_internal)
        .def("__copy__",  [](const Point3D &self) {
            return Point3D(self);
        })
        .def("__deepcopy__",  [](const Point3D &self, py::dict) {
            return Point3D(self);
        })
        .def("__repr__", [](const Point3D &self) {
            std::stringstream ss;
            ss<<"<Point3D 'xyz=["
                <<self.XYZ().transpose()
                <<"], track_length=" 
                <<(self.Track().Length())
                <<", error="
                <<self.Error()<<"'>";
            return ss.str();
        })
        .def("summary", [](const Point3D &self) {
            std::stringstream ss;
            ss<<"<Point3D:\n\txyz = ["
                <<self.XYZ().transpose()
                <<"]\n\ttrack_length = " 
                <<(self.Track().Length())
                <<"\n\terror = "
                <<self.Error()
                <<"\n\tcolor = ["
                <<self.Color().cast<int>().transpose()
                <<"]";
            return ss.str();
        });

    py::class_<colmap::Image, std::shared_ptr<colmap::Image>>(m, "Image")
        .def(py::init<>())
        .def(py::init([](const std::string& name,
                         const std::vector<Eigen::Vector2d>& keypoints,
                         const Eigen::Vector3d& tvec,
                         const Eigen::Vector4d& qvec,
                         size_t camera_id){
            std::unique_ptr<Image> image = std::unique_ptr<Image>(new Image());
            image->SetName(name);
            image->SetPoints2D(keypoints);
            image->SetTvec(tvec);
            image->SetQvec(qvec);
            if (camera_id != kInvalidCameraId) {
                image->SetCameraId(camera_id);
            }
            return image;
        }), py::arg("name"), py::arg("keypoints") = std::vector<Eigen::Vector2d>(), 
            py::arg("tvec") = Eigen::Vector3d(0.0,0.0,0.0),
            py::arg("qvec") = Eigen::Vector4d(1.0,0.0,0.0,0.0), 
            py::arg("camera_id_id") = kInvalidCameraId)
        .def(py::init([](const std::string& name,
                         const std::vector<Point2D>& points2D,
                         const Eigen::Vector3d& tvec,
                         const Eigen::Vector4d& qvec,
                         size_t camera_id){
            std::unique_ptr<Image> image = std::unique_ptr<Image>(new Image());
            image->SetName(name);
            image->SetPoints2D(points2D);
            image->SetTvec(tvec);
            image->SetQvec(qvec);
            if (camera_id != kInvalidCameraId) {
                image->SetCameraId(camera_id);
            }
            return image;
        }), py::arg("name"), py::arg("points2D") = std::vector<Point2D>(), 
            py::arg("tvec") = Eigen::Vector3d(0.0,0.0,0.0), 
            py::arg("qvec") = Eigen::Vector4d(1.0,0.0,0.0,0.0), 
            py::arg("camera_id_id") = kInvalidCameraId)
        .def_property("image_id", &Image::ImageId, &Image::SetImageId, "Unique identifier of image.")
        .def_property("camera_id", &Image::CameraId, [](Image& self, const camera_t camera_id) {
                THROW_CHECK_NE(camera_id, kInvalidCameraId);
                self.SetCameraId(kInvalidCameraId);
        }, "Unique identifier of the camera.")
        .def_property("name", overload_cast_<>()(&Image::Name), &Image::SetName, "Name of the image.")
        .def_property("qvec", overload_cast_<>()(&Image::Qvec), &Image::SetQvec, 
                "Quaternion vector (qw,qx,qy,qz) which describes rotation from world to image space.")
        .def_property("tvec", overload_cast_<>()(&Image::Tvec), &Image::SetTvec,
                "Translation vector (tx,ty,tz) which describes translation from world to image space.")
        .def_property("qvec_prior", overload_cast_<>()(&Image::QvecPrior), &Image::SetQvecPrior,
                "Quaternion prior, e.g. given by EXIF gyroscope tag.")
        .def_property("tvec_prior", overload_cast_<>()(&Image::TvecPrior), &Image::SetTvecPrior,
                "Translation prior, e.g. given by EXIF GPS tag.")
        .def_property("points2D", &Image::Points2D, 
            [](Image& self, const std::vector<class Point2D>& points2D) {
                 THROW_CUSTOM_CHECK(!points2D.empty(), std::invalid_argument);
                 self.SetPoints2D(points2D);   
            },"Array of Points2D (=keypoints).")
        .def("set_point3D_for_point2D", [](
                Image& self, 
                const point2D_t point2D_idx,
                const point3D_t point3D_id) {
                THROW_CHECK_NE(point3D_id, kInvalidPoint3DId);
                self.SetPoint3DForPoint2D(point2D_idx, point3D_id);
        },     "Set the point as triangulated, i.e. it is part of a 3D point track.")
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
        .def("normalize_qvec",&Image::NormalizeQvec,
                "Normalize the quaternion vector.")
        .def("projection_matrix", &Image::ProjectionMatrix,
                "Compose the projection matrix from world to image space.")
        .def("inverse_projection_matrix", &Image::InverseProjectionMatrix,
                "Compose the inverse projection matrix from image to world space.")
        .def("projection_center", &Image::ProjectionCenter,
                "Extract the projection center in world space.")
        .def("viewing_direction", &Image::ViewingDirection,
                "Extract the viewing direction of the image.")
        .def("rotation_matrix", &Image::RotationMatrix,
                "Compose rotation matrix from quaternion vector.")
        .def("rotmat", &Image::RotationMatrix,
                "Compose rotation matrix from quaternion vector.")
        .def("set_up", [](Image& self, const class Camera& camera) {
                   THROW_CHECK_EQ(self.CameraId(), camera.CameraId());
                   self.SetUp(camera);
                },
                "Setup the image and necessary internal data structures before being used in reconstruction.")
        .def("has_camera", &Image::HasCamera,
                "Check whether identifier of camera has been set.")
        .def_property("registered", &Image::IsRegistered, &Image::SetRegistered,
                "Whether image is registered in the reconstruction.")
        .def("num_points2D", &Image::NumPoints2D,
                "Get the number of image points (keypoints).")
        .def("num_points3D", &Image::NumPoints3D,
                "Get the number of triangulations, i.e. the number of points that\n"
                "are part of a 3D point track.")
        .def_property("num_observations", &Image::NumObservations, &Image::SetNumObservations,
                "Number of observations, i.e. the number of image points that\n"
                "have at least one correspondence to another image.")
        .def_property("num_correspondences", &Image::NumCorrespondences, &Image::SetNumCorrespondences,
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
        .def("get_valid_point2D_ids", [](const Image& self) {
            std::vector<colmap::point2D_t> valid_point2D_ids;

            for (colmap::point2D_t point2D_idx = 0; point2D_idx < self.NumPoints2D();
                ++point2D_idx) {
                if (self.Point2D(point2D_idx).HasPoint3D()) {
                    valid_point2D_ids.push_back(point2D_idx);
                }
            }

            return valid_point2D_ids;
        })
        .def("get_valid_points2D", [](const Image& self) {
            std::vector<colmap::Point2D> valid_points2D;

            for (colmap::point2D_t point2D_idx = 0; point2D_idx < self.NumPoints2D();
                ++point2D_idx) {
                if (self.Point2D(point2D_idx).HasPoint3D()) {
                    valid_points2D.push_back(self.Point2D(point2D_idx));
                }
            }

            return valid_points2D;
        })
        .def("project", [](const Image& self, const Eigen::Vector3d& world){
            const Eigen::Vector3d world_point = self.ProjectionMatrix() * world.homogeneous();
            return world_point.hnormalized();
        }, "Project world point to image coordinate frame.")
        .def("project", [](const Image& self, const std::vector<Eigen::Vector3d>& world_coords){
            const Eigen::Matrix3x4d projection_matrix = self.ProjectionMatrix();
            std::vector<Eigen::Vector2d> world_points(world_coords.size());
            for (int idx = 0; idx < world_coords.size(); ++idx) {
                world_points[idx] = (projection_matrix * world_coords[idx].homogeneous()).hnormalized();
            }
            return world_points;
        }, "Project list of world points to image coordinate frame.")
        .def("project", [](const Image& self, const std::vector<Point3D>& point3Ds){
            const Eigen::Matrix3x4d projection_matrix = self.ProjectionMatrix();
            std::vector<Eigen::Vector2d> world_points(point3Ds.size());
            for (int idx = 0; idx < point3Ds.size(); ++idx) {
                world_points[idx] = (projection_matrix * point3Ds[idx].XYZ().homogeneous()).hnormalized();
            }
            return world_points;
        }, "Project list world points to image coordinate frame.")
        .def("__copy__",  [](const Image &self) {
            return Image(self);
        })
        .def("__deepcopy__",  [](const Image &self, py::dict) {
            return Image(self);
        })
        .def("__repr__", [](const Image &self) {
            std::stringstream ss;
            ss<<"<Image 'image_id="
                <<(self.ImageId() != kInvalidImageId ? std::to_string(self.ImageId()) : "Invalid")
                <<", camera_id=" 
                <<(self.HasCamera() ? std::to_string(self.CameraId()) : "Invalid")
                <<", name="
                <<self.Name()
                <<", triangulated="
                <<self.NumPoints3D()<<"/"<<self.NumPoints2D()
                <<"'>";
            return ss.str();
        })
        .def("summary", [](const Image &self) {
            std::stringstream ss;
            ss<<"Image:\n\timage_id = "
                <<(self.ImageId() != kInvalidImageId ? std::to_string(self.ImageId()) : "Invalid")
                <<"\n\tcamera_id = " 
                <<(self.HasCamera() ? std::to_string(self.CameraId()) : "Invalid")
                <<"\n\tname = "
                <<self.Name()
                <<"\n\ttriangulated = "
                <<self.NumPoints3D()<<"/"<<self.NumPoints2D()
                <<"\n\ttvec = ["
                <<self.Tvec().transpose()
                <<"]\n\tqvec = ["
                <<self.Qvec().transpose()
                <<"]";
            return ss.str();
        });

    py::class_<Camera, std::shared_ptr<Camera>>(m, "Camera")
        .def(py::init<>())
        .def(py::init([](const std::string& name, 
                         size_t width, 
                         size_t height, 
                         const std::vector<double>& params){
                std::unique_ptr<Camera> camera = std::unique_ptr<Camera>(new Camera());
                THROW_CHECK(ExistsCameraModelWithName(name));
                camera->SetModelIdFromName(name);
                camera->SetWidth(width);
                camera->SetHeight(height);
                camera->SetParams(params);
                return camera;
        }))
        .def_property("camera_id", &Camera::CameraId, &Camera::SetCameraId,
                "Unique identifier of the camera.")
        .def_property("model_id", &Camera::ModelId, [](Camera& self,int model_id){
                        THROW_CHECK(ExistsCameraModelWithId(model_id));
                        self.SetModelId(model_id);
                },
                "Camera model ID.")
        .def_property("model_name", &Camera::ModelName, [](
                                Camera& self,
                                std::string model_name){
                        THROW_CHECK(ExistsCameraModelWithName(model_name));
                        self.SetModelIdFromName(model_name);
                },"Camera model name (connected to model_id).")
        .def_property("width", &Camera::Width, &Camera::SetWidth,
                "Width of camera sensor.")
        .def_property("height", &Camera::Height, &Camera::SetHeight,
                "Height of camera sensor.")
        .def("mean_focal_length", &Camera::MeanFocalLength)
        .def_property("focal_length", &Camera::FocalLength, &Camera::SetFocalLength)
        .def_property("focal_length_x", &Camera::FocalLengthX, &Camera::SetFocalLengthX)
        .def_property("focal_length_y", &Camera::FocalLengthY, &Camera::SetFocalLengthY)
        .def_property("principal_point_x",&Camera::PrincipalPointX, &Camera::SetPrincipalPointX)
        .def_property("principal_point_y",&Camera::PrincipalPointY, &Camera::SetPrincipalPointY)
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
        .def("num_params", &Camera::NumParams,
                "Number of raw camera parameters.")
        .def_property("params",overload_cast_<>()(&Camera::Params), &Camera::SetParams,
                "Camera parameters.")
        .def("params_to_string",&Camera::ParamsToString,
                "Concatenate parameters as comma-separated list.")
        .def("set_params_from_string",&Camera::SetParamsFromString,
                "Set camera parameters from comma-separated list.")
        .def("verify_params",&Camera::VerifyParams,
                "Check whether parameters are valid, i.e. the parameter vector has\n"
                "the correct dimensions that match the specified camera model.")
        .def("has_bogus_params", &Camera::HasBogusParams,
                "Check whether camera has bogus parameters.")
        .def("initialize_with_id", [](Camera& self, const int model_id, 
        const double focal_length, const size_t width, const size_t height){
                THROW_CHECK(ExistsCameraModelWithId(model_id));
                self.InitializeWithId(model_id, focal_length, width, height);
            },"Initialize parameters for given camera model ID and focal length, and set\n"
                "the principal point to be the image center.")
        .def("initialize_with_name", [](Camera& self, std::string model_name, 
            const double focal_length, const size_t width, const size_t height){
                THROW_CHECK(ExistsCameraModelWithName(model_name));
                self.InitializeWithName(model_name, focal_length, width, height);
            },
                "Initialize parameters for given camera model name and focal length, and set\n"
                "the principal point to be the image center.")
        .def("image_to_world", &Camera::ImageToWorld,
                "Project point in image plane to world / infinity.")
        .def("image_to_world", [](const Camera& self, 
                                  const std::vector<Eigen::Vector2d>& points2D) {
                std::vector<Eigen::Vector2d> world_points2D;
                for (size_t idx = 0; idx < points2D.size(); ++idx) {
                    world_points2D.push_back(self.ImageToWorld(points2D[idx]));
                }
                return world_points2D;
            }, "Project list of points in image plane to world / infinity.")
        .def("image_to_world", [](const Camera& self, 
                                  const std::vector<Point2D>& points2D) {
                std::vector<Eigen::Vector2d> world_points2D;
                for (size_t idx = 0; idx < points2D.size(); ++idx) {
                    world_points2D.push_back(self.ImageToWorld(points2D[idx].XY()));
                }
                return world_points2D;
            }, "Project list of points in image plane to world / infinity.")
        .def("image_to_world_threshold", &Camera::ImageToWorldThreshold,
                "Convert pixel threshold in image plane to world space.")
        .def("world_to_image", &Camera::WorldToImage,
                "Project point from world / infinity to image plane.")
        .def("world_to_image", [](const Camera& self, 
                                  const std::vector<Eigen::Vector2d>& world_points2D) {
            std::vector<Eigen::Vector2d> image_points2D;
            for (size_t idx = 0; idx < world_points2D.size(); ++idx) {
                image_points2D.push_back(self.WorldToImage(world_points2D[idx]));
            }
            return image_points2D;
        }, "Project list of points from world / infinity to image plane.")
        .def("world_to_image", [](const Camera& self, 
                                  const std::vector<Point2D>& world_points2D) {
            std::vector<Eigen::Vector2d> image_points2D;
            for (size_t idx = 0; idx < world_points2D.size(); ++idx) {
                image_points2D.push_back(self.WorldToImage(world_points2D[idx].XY()));
            }
            return image_points2D;
        }, "Project list of points from world / infinity to image plane.")
        .def("rescale", overload_cast_<size_t,size_t>()(&Camera::Rescale),
                "Rescale camera dimensions to (width_height) and accordingly the focal length and\n"
                "and the principal point.")
        .def("rescale", overload_cast_<double>()(&Camera::Rescale),
                "Rescale camera dimensions by given factor and accordingly the focal length and\n"
                "and the principal point.")
        .def("__copy__",  [](const Camera &self) {
            return Camera(self);
        })
        .def("__deepcopy__",  [](const Camera &self, py::dict) {
            return Camera(self);
        })
        .def("__repr__", [](const Camera &self) {
            std::stringstream ss;
            ss<<"<Camera 'camera_id=" 
                <<(self.CameraId()!=kInvalidCameraId ? 
                    std::to_string(self.CameraId()) : "Invalid")
                <<", model="<<self.ModelName()
                <<", width="<<self.Width()
                <<", height="<<self.Height()
                <<", num_params="<<self.NumParams()
                <<"'>";
            return ss.str();
        })
        .def("summary", [](const Camera &self) {
            std::stringstream ss;
            ss<<"Camera:\n\tcamera_id=" 
                <<(self.CameraId()!=kInvalidCameraId ? 
                    std::to_string(self.CameraId()) : "Invalid")
                <<"\n\tmodel = "<<self.ModelName()
                <<"\n\twidth = "<<self.Width()
                <<"\n\theight = "<<self.Height()
                <<"\n\tnum_params = "<<self.NumParams()
                <<"\n\tparams_info = "<<self.ParamsInfo()
                <<"\n\tparams = "<<self.ParamsToString();
            return ss.str();
        });

    py::class_<Reconstruction>(m, "Reconstruction")
        .def(py::init<>())
        .def(py::init([](const std::string& input_path){
            THROW_CUSTOM_CHECK_MSG(
                ExistsReconstruction(input_path),
                std::invalid_argument,
                (std::string("cameras, images, points3D not found at ")
                    +input_path).c_str()
            );
            auto reconstruction = std::unique_ptr<Reconstruction>(new Reconstruction());
            reconstruction->Read(input_path);
            return reconstruction;
        }))
        .def("read", [](Reconstruction& self, const std::string& input_path) {
            THROW_CUSTOM_CHECK_MSG(
                ExistsReconstruction(input_path),
                std::invalid_argument,
                (std::string("cameras, images, points3D not found at ")
                    +input_path).c_str()
            );
            self.Read(input_path);
        }, "Read reconstruction in COLMAP format. Prefer binary.")
        .def("write", [](const Reconstruction& self,const std::string& path) {
            THROW_CUSTOM_CHECK_MSG(
                ExistsDir(path),
                std::invalid_argument,
                (std::string("Directory ")+ path + " does not exist.").c_str()
            );
            self.Write(path);
        }, "Write reconstruction in COLMAP binary format.")
        .def("num_images", &Reconstruction::NumImages)
        .def("num_cameras", &Reconstruction::NumImages)
        .def("num_reg_images", &Reconstruction::NumImages)
        .def("num_points3D", &Reconstruction::NumImages)
        .def("num_image_pairs", &Reconstruction::NumImagePairs)
        .def_property_readonly("images", &Reconstruction::Images, py::return_value_policy::reference)
        .def_property_readonly("image_pairs", &Reconstruction::ImagePairs)
        .def_property_readonly("cameras", &Reconstruction::Cameras, py::return_value_policy::reference)
        .def_property_readonly("points3D", &Reconstruction::Points3D, py::return_value_policy::reference)
        .def("point3D_ids", &Reconstruction::Point3DIds)
        .def("reg_image_ids", &Reconstruction::RegImageIds)
        .def("exists_camera", &Reconstruction::ExistsCamera)
        .def("exists_image", &Reconstruction::ExistsImage)
        .def("exists_point3D", &Reconstruction::ExistsPoint3D)
        .def("exists_image_pair", &Reconstruction::ExistsImagePair)
        // .def("image", overload_cast_<image_t>()(&Reconstruction::Image, py::const_))
        // .def("camera", overload_cast_<camera_t>()(&Reconstruction::Camera, py::const_)) 
        // .def("point3D", overload_cast_<point3D_t>()(&Reconstruction::Point3D, py::const_))
        .def("add_camera", &Reconstruction::AddCamera,
                "Add new camera. There is only one camera per image, while multiple images\n"
                "might be taken by the same camera.")
        .def("add_image", &Reconstruction::AddImage,
                "Add new image.")
        .def("add_point3D", &Reconstruction::AddPoint3D,
                "Add new 3D object, and return its unique ID.")
        .def("add_observation", &Reconstruction::AddObservation,
                "Add observation to existing 3D point.")
        .def("merge_points3D", &Reconstruction::MergePoints3D,
                "Merge two 3D points and return new identifier of new 3D point.\n"
                "The location of the merged 3D point is a weighted average of the two\n"
                "original 3D point's locations according to their track lengths.")
        .def("delete_point3D", &Reconstruction::DeletePoint3D,
                "Delete a 3D point, and all its references in the observed images.")
        .def("delete_observation", &Reconstruction::DeleteObservation,
                "Delete one observation from an image and the corresponding 3D point.\n"
                "Note that this deletes the entire 3D point, if the track has two elements\n"
                "prior to calling this method.")
        .def("register_image", &Reconstruction::RegisterImage,
                "Register an existing image.")
        .def("deregister_image", &Reconstruction::DeRegisterImage,
                "De-register an existing image, and all its references.")
        .def("is_image_registered", &Reconstruction::IsImageRegistered,
                "Check if image is registered.")
        .def("normalize", &Reconstruction::Normalize,
                "Normalize scene by scaling and translation to avoid degenerate\n"
                "visualization after bundle adjustment and to improve numerical\n"
                "stability of algorithms.\n\n"
                "Translates scene such that the mean of the camera centers or point\n"
                "locations are at the origin of the coordinate system.\n\n"
                "Scales scene such that the minimum and maximum camera centers are at the\n"
                "given `extent`, whereas `p0` and `p1` determine the minimum and\n"
                "maximum percentiles of the camera centers considered.")
        // .def("transform", &Reconstruction::Transform)
        .def("merge", &Reconstruction::Merge,
                "Merge the given reconstruction into this reconstruction by registering the\n"
                "images registered in the given but not in this reconstruction and by\n"
                "merging the two clouds and their tracks. The coordinate frames of the two\n"
                "reconstructions are aligned using the projection centers of common\n"
                "registered images. Return true if the two reconstructions could be merged.")
        // We do not add custom checks atm for align since function headers are different between
        // COLMAP versions. Thus, this could potentially lead to fatal errors (but checks are obvious).
        .def("align", &Reconstruction::Align,
                "Align the given reconstruction with a set of pre-defined camera positions.\n"
                "Assuming that locations[i] gives the 3D coordinates of the center\n"
                "of projection of the image with name image_names[i].")
        .def("align_robust", &Reconstruction::AlignRobust,
                "Robust alignment using RANSAC.")
        .def("find_image_with_name", &Reconstruction::FindImageWithName, 
                                py::return_value_policy::reference_internal,
            "Find image with matching name. Returns None if no match is found.")
        .def("find_common_reg_image_ids", &Reconstruction::FindCommonRegImageIds,
                "Find images that are both present in this and the given reconstruction.")
        .def("filter_points3D", &Reconstruction::FilterPoints3D,
                "Filter 3D points with large reprojection error, negative depth, or\n"
                "insufficient triangulation angle.\n\n"
                "@param max_reproj_error    The maximum reprojection error.\n"
                "@param min_tri_angle       The minimum triangulation angle.\n"
                "@param point3D_ids         The points to be filtered.\n\n"
                "@return                    The number of filtered observations.")
        .def("filter_points3D_in_images", &Reconstruction::FilterPoints3DInImages,
                "Filter 3D points with large reprojection error, negative depth, or\n"
                "insufficient triangulation angle.\n\n"
                "@param max_reproj_error    The maximum reprojection error.\n"
                "@param min_tri_angle       The minimum triangulation angle.\n"
                "@param image_ids           The the image ids in which the points3D are filtered.\n\n"
                "@return                    The number of filtered observations.")
        .def("filter_all_points3D", &Reconstruction::FilterAllPoints3D,
                "Filter 3D points with large reprojection error, negative depth, or\n"
                "insufficient triangulation angle.\n\n"
                "@param max_reproj_error    The maximum reprojection error.\n"
                "@param min_tri_angle       The minimum triangulation angle.\n\n"
                "@return                    The number of filtered observations.")
        .def("filter_observations_with_negative_depth", &Reconstruction::FilterObservationsWithNegativeDepth,
                "Filter observations that have negative depth.\n\n"
                "@return    The number of filtered observations.")
        .def("filter_images", &Reconstruction::FilterImages,
                "Filter images without observations or bogus camera parameters.\n\n"
                "@return    The identifiers of the filtered images.")
        .def("compute_num_observations", &Reconstruction::ComputeNumObservations)
        .def("compute_mean_track_length", &Reconstruction::ComputeMeanTrackLength)
        .def("compute_mean_observations_per_reg_image", &Reconstruction::ComputeMeanObservationsPerRegImage)
        .def("compute_mean_reprojection_error", &Reconstruction::ComputeMeanReprojectionError)
        .def("read_text", [](Reconstruction& self, const std::string& input_path) {
            THROW_CUSTOM_CHECK_MSG(
                ExistsReconstructionText(input_path),
                std::invalid_argument,
                (std::string("cameras.txt, images.txt, points3D.txt not found at ")
                    +input_path).c_str()
            );
            self.ReadText(input_path);
        })
        .def("read_binary", [](Reconstruction& self, const std::string& input_path) {
            THROW_CUSTOM_CHECK_MSG(
                ExistsReconstructionBinary(input_path),
                std::invalid_argument,
                (std::string("cameras.bin, images.bin, points3D.bin not found at ")
                    +input_path).c_str()
            );
            self.ReadBinary(input_path);
        })
        .def("write_text", [](const Reconstruction& self,const std::string& path) {
            THROW_CUSTOM_CHECK_MSG(
                ExistsDir(path),
                std::invalid_argument,
                (std::string("Directory ")+ path + " does not exist.").c_str()
            );
            self.WriteText(path);
        })
        .def("write_binary", [](const Reconstruction& self,const std::string& path) {
            THROW_CUSTOM_CHECK_MSG(
                ExistsDir(path),
                std::invalid_argument,
                (std::string("Directory ")+ path + " does not exist.").c_str()
            );
            self.WriteBinary(path);
        })
        .def("convert_to_PLY", &Reconstruction::ConvertToPLY)
        .def("import_PLY", &Reconstruction::ImportPLY,
                "Import from PLY format. Note that these import functions are\n"
                "only intended for visualization of data and usable for reconstruction.")
        .def("export_NVM", &Reconstruction::ExportNVM,
                "Export reconstruction in NVM format.\n"
                "WARNING: Can raise fatal error if the file path cannot be opened.")
        .def("export_bundler", &Reconstruction::ExportBundler,
                "Export reconstruction in Bundler format.\n"
                "WARNING: Can raise fatal error if the file paths cannot be opened.")
        .def("export_PLY", [](const Reconstruction& self, 
            const std::string& path) {
               std::ofstream file(path, std::ios::trunc);
                THROW_CUSTOM_CHECK_MSG(file.is_open(),
                std::invalid_argument,
                (std::string(": Could not open ") + path).c_str());
                file.close();
                self.ExportPLY(path);
            }, "Export reconstruction in PLY format.")
        .def("export_VRML", [](const Reconstruction& self, 
                const std::string& images_path,
                const std::string& points3D_path,
                const double image_scale,
                const Eigen::Vector3d& image_rgb) {
                    std::ofstream image_file(images_path, std::ios::trunc);
                    THROW_CUSTOM_CHECK_MSG(image_file.is_open(),
                    std::invalid_argument,
                    (std::string(": Could not open ") + images_path).c_str());
                    image_file.close();

                    std::ofstream p3D_file(points3D_path, std::ios::trunc);
                    THROW_CUSTOM_CHECK_MSG(p3D_file.is_open(),
                    std::invalid_argument,
                    (std::string(": Could not open ") + points3D_path).c_str());
                    p3D_file.close();

                    self.ExportVRML(images_path, points3D_path,
                        image_scale, image_rgb);
                }, "Export reconstruction in VRML format.")
        .def("extract_colors_for_image", &Reconstruction::ExtractColorsForImage,
                "Extract colors for 3D points of given image. Colors will be extracted\n"
                "only for 3D points which are completely black.\n\n"
                "@param image_id      Identifier of the image for which to extract colors.\n"
                "@param path          Absolute or relative path to root folder of image.\n"
                "                     The image path is determined by concatenating the\n"
                "                     root path and the name of the image.\n\n"
                "@return              True if image could be read at given path.")
        .def("extract_colors_for_all_images", &Reconstruction::ExtractColorsForAllImages,
                "Extract colors for all 3D points by computing the mean color of all images.\n\n"
                "@param path          Absolute or relative path to root folder of image.\n"
                "                     The image path is determined by concatenating the\n"
                "                     root path and the name of the image.")
        .def("create_image_dirs", &Reconstruction::CreateImageDirs,
                "Create all image sub-directories in the given path.")
        .def("__copy__",  [](const Reconstruction &self) {
            return Reconstruction(self);
        })
        .def("__deepcopy__",  [](const Reconstruction &self, py::dict) {
            return Reconstruction(self);
        })
        .def("__repr__", [](const Reconstruction &self) {
            std::stringstream ss;
            ss<<"<Reconstruction 'num_reg_images="<<self.NumRegImages()
                <<", num_cameras="<<self.NumCameras()
                <<", num_points3D="<<self.NumPoints3D()
                <<", num_observations="<<self.ComputeNumObservations()<<"'>";
            return ss.str();
        })
        .def("summary", [](const Reconstruction &self) {
            std::stringstream ss;
            ss<<"Reconstruction:"
                <<"\n\tnum_reg_images = "
                <<self.NumRegImages()
                <<"\n\tnum_cameras = "
                <<self.NumCameras()
                <<"\n\tnum_points3D = "
                <<self.NumPoints3D()
                <<"\n\tnum_observations = "
                <<self.ComputeNumObservations()
                <<"\n\tmean_track_length = "
                <<self.ComputeMeanTrackLength()
                <<"\n\tmean_observations_per_image = "
                <<self.ComputeMeanObservationsPerRegImage()
                <<"\n\tmean_reprojection_error = "
                <<self.ComputeMeanReprojectionError();
            return ss.str();
        });
}