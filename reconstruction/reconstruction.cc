// Author: Philipp Lindenberger (Phil26AT)

#include "colmap/base/reconstruction.h"
#include "colmap/base/camera_models.h"
#include "colmap/base/projection.h"
#include "colmap/util/misc.h"
#include "colmap/util/ply.h"
#include "colmap/util/types.h"

using namespace colmap;

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;
using namespace pybind11::literals;

#include "log_exceptions.h"
#include "reconstruction/camera.cc"
#include "reconstruction/image.cc"
#include "reconstruction/point2D.cc"
#include "reconstruction/point3D.cc"
#include "reconstruction/track.cc"
#include "reconstruction/utils.cc"

template <typename... Args>
using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;

void init_track(py::module&);
void init_point2D(py::module&);
void init_point3D(py::module&);
void init_image(py::module&);
void init_camera(py::module&);
void init_reconstruction_utils(py::module&);

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

#define THROW_CHECK_RECONSTRUCTION_TEXT_EXISTS(input_path)           \
    THROW_CUSTOM_CHECK_MSG(                                          \
        ExistsReconstructionText(input_path), std::invalid_argument, \
        std::string("cameras.txt, images.txt, points3D.txt not found at ") + input_path);

#define THROW_CHECK_RECONSTRUCTION_BIN_EXISTS(input_path)              \
    THROW_CUSTOM_CHECK_MSG(                                            \
        ExistsReconstructionBinary(input_path), std::invalid_argument, \
        std::string("cameras.bin, images.bin, points3D.bin not found at ") + input_path);

#define THROW_CHECK_RECONSTRUCTION_EXISTS(input_path)                               \
    THROW_CUSTOM_CHECK_MSG(ExistsReconstruction(input_path), std::invalid_argument, \
                           std::string("cameras, images, points3D not found at ") + input_path);

// Reconstruction Bindings
void init_reconstruction(py::module& m) {
    // STL Containers, required for fast looping over members (avoids copying)
    using ImageMap = EIGEN_STL_UMAP(colmap::image_t, colmap::Image);
    using Point3DMap = EIGEN_STL_UMAP(colmap::point3D_t, colmap::Point3D);
    using CameraMap = EIGEN_STL_UMAP(colmap::camera_t, colmap::Camera);

    init_track(m);
    init_point2D(m);
    init_point3D(m);
    init_image(m);
    init_camera(m);
    init_reconstruction_utils(m);

    py::class_<Reconstruction>(m, "Reconstruction")
        .def(py::init<>())
        .def(py::init([](const py::object input_path) {
                 std::string path = py::str(input_path).cast<std::string>();
                 THROW_CHECK_RECONSTRUCTION_EXISTS(path);
                 auto reconstruction = std::unique_ptr<Reconstruction>(new Reconstruction());
                 reconstruction->Read(path);
                 return reconstruction;
             }),
             py::arg("sfm_dir"))
        .def(
            "read",
            [](Reconstruction& self, py::object input_path) {
                std::string path = py::str(input_path).cast<std::string>();
                THROW_CHECK_RECONSTRUCTION_EXISTS(path);
                self.Read(path);
            },
            py::arg("sfm_dir"), "Read reconstruction in COLMAP format. Prefer binary.")
        .def(
            "write",
            [](const Reconstruction& self, py::object output_path) {
                std::string path = py::str(output_path).cast<std::string>();
                THROW_CHECK_DIR_EXISTS(path);
                self.Write(path);
            },
            py::arg("output_dir"), "Write reconstruction in COLMAP binary format.")
        .def("read_text",
             [](Reconstruction& self, const std::string& input_path) {
                 THROW_CHECK_RECONSTRUCTION_TEXT_EXISTS(input_path);
                 self.ReadText(input_path);
             })
        .def("read_binary",
             [](Reconstruction& self, const std::string& input_path) {
                 THROW_CHECK_RECONSTRUCTION_BIN_EXISTS(input_path);
                 self.ReadBinary(input_path);
             })
        .def("write_text",
             [](const Reconstruction& self, const std::string& path) {
                 THROW_CHECK_DIR_EXISTS(path);
                 self.WriteText(path);
             })
        .def("write_binary",
             [](const Reconstruction& self, const std::string& path) {
                 THROW_CHECK_DIR_EXISTS(path);
                 self.WriteBinary(path);
             })
        .def("num_images", &Reconstruction::NumImages)
        .def("num_cameras", &Reconstruction::NumCameras)
        .def("num_reg_images", &Reconstruction::NumRegImages)
        .def("num_points3D", &Reconstruction::NumPoints3D)
        .def("num_image_pairs", &Reconstruction::NumImagePairs)
        .def_property_readonly("images", &Reconstruction::Images,
                               py::return_value_policy::reference)
        .def_property_readonly("image_pairs", &Reconstruction::ImagePairs)
        .def_property_readonly("cameras", &Reconstruction::Cameras,
                               py::return_value_policy::reference)
        .def_property_readonly("points3D", &Reconstruction::Points3D,
                               py::return_value_policy::reference)
        .def("point3D_ids", &Reconstruction::Point3DIds)
        .def("reg_image_ids", &Reconstruction::RegImageIds)
        .def("exists_camera", &Reconstruction::ExistsCamera)
        .def("exists_image", &Reconstruction::ExistsImage)
        .def("exists_point3D", &Reconstruction::ExistsPoint3D)
        .def("exists_image_pair", &Reconstruction::ExistsImagePair)
        .def(
            "add_camera",
            [](Reconstruction& self, const class colmap::Camera& camera) {
                THROW_CHECK(!self.ExistsCamera(camera.CameraId()));
                THROW_CHECK(camera.VerifyParams());
                self.AddCamera(camera);
            },
            "Add new camera. There is only one camera per image, while multiple images\n"
            "might be taken by the same camera.")
        .def(
            "add_image",
            [](Reconstruction& self, const class colmap::Image& image, bool check_not_registered) {
                THROW_CHECK(!self.ExistsImage(image.ImageId()));
                if (check_not_registered) {
                    THROW_CHECK(!image.IsRegistered());
                }
                self.AddImage(image);
                if (image.IsRegistered()) {
                    THROW_CHECK_NE(image.ImageId(), colmap::kInvalidImageId);
                    self.Image(image.ImageId()).SetRegistered(false);  // Set true in next line
                    self.RegisterImage(image.ImageId());
                }
            },
            py::arg("image"), py::arg("check_not_registered") = false,
            "Add new image. If Image.IsRegistered()==true, either throw "
            "if check_not_registered, or register image also in reconstr.")
        .def("add_point3D", &Reconstruction::AddPoint3D,
             "Add new 3D object, and return its unique ID.",
             py::arg("xyz"), py::arg("track"), py::arg("color") = Eigen::Vector3ub::Zero())
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
        .def(
            "register_image",
            [](colmap::Reconstruction& self, colmap::image_t imid) {
                THROW_CHECK_EQ(self.Image(imid).IsRegistered(), self.IsImageRegistered(imid));
                self.RegisterImage(imid);
            },
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
        .def(
            "transform",
            [](Reconstruction& self, const Eigen::Matrix3x4d& tform_mat) {
                SimilarityTransform3 tform(tform_mat);
                self.Transform(tform);
            },
            "Apply the 3D similarity transformation to all images and points.")
        .def("transform", &Reconstruction::Transform,
             "Apply the 3D similarity transformation to all images and points.")
        .def("merge", &Reconstruction::Merge,
             "Merge the given reconstruction into this reconstruction by registering the\n"
             "images registered in the given but not in this reconstruction and by\n"
             "merging the two clouds and their tracks. The coordinate frames of the two\n"
             "reconstructions are aligned using the projection centers of common\n"
             "registered images. Return true if the two reconstructions could be merged.")
        .def("align_poses", &AlignPosesBetweenReconstructions, py::arg("ref_reconstruction"),
             py::arg("min_inlier_observations") = 0.3, py::arg("max_reproj_error") = 8.0,
             "Align to reference reconstruction using RANSAC.")
        .def("align_points", &AlignPointsBetweenReconstructions,
             py::arg("reference_reconstruction"), py::arg("min_overlap") = 3,
             py::arg("max_error") = 0.005, py::arg("min_inlier_ratio") = 0.9,
             "Align 3D points to reference reconstruction using LORANSAC.\n"
             "Estimates pose by aligning corresponding 3D points.\n"
             "Correspondences are estimated by counting similar detection idxs.\n"
             "Assumes image_ids and point2D_idx overlap.")
        .def("align", &AlignReconstruction,
             "Align the given reconstruction with a set of pre-defined camera positions.\n"
             "Assuming that locations[i] gives the 3D coordinates of the center\n"
             "of projection of the image with name image_names[i].")
        .def("align_robust", &RobustAlignReconstruction, py::arg("image_names"),
             py::arg("locations"), py::arg("min_common_images"), py::arg("max_error") = 12.0,
             py::arg("min_inlier_ratio") = 0.1,
             "Robust alignment using RANSAC. \n"
             "Align the given reconstruction with a set of pre-defined camera positions.\n"
             "Assuming that locations[i] gives the 3D coordinates of the center\n"
             "of projection of the image with name image_names[i].")
        .def("compute_bounding_box", &Reconstruction::ComputeBoundingBox, py::arg("p0") = 0.0,
             py::arg("p1") = 1.0)
        .def("crop", &Reconstruction::Crop)
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
        .def("filter_observations_with_negative_depth",
             &Reconstruction::FilterObservationsWithNegativeDepth,
             "Filter observations that have negative depth.\n\n"
             "@return    The number of filtered observations.")
        .def("filter_images", &Reconstruction::FilterImages,
             "Filter images without observations or bogus camera parameters.\n\n"
             "@return    The identifiers of the filtered images.")
        .def("compute_num_observations", &Reconstruction::ComputeNumObservations)
        .def("compute_mean_track_length", &Reconstruction::ComputeMeanTrackLength)
        .def("compute_mean_observations_per_reg_image",
             &Reconstruction::ComputeMeanObservationsPerRegImage)
        .def("compute_mean_reprojection_error", &Reconstruction::ComputeMeanReprojectionError)
        // .def("convert_to_PLY", &Reconstruction::ConvertToPLY)
        .def("import_PLY", overload_cast_<const std::string&>()(&Reconstruction::ImportPLY),
             "Import from PLY format. Note that these import functions are\n"
             "only intended for visualization of data and usable for reconstruction.")
        .def("export_NVM",
            [](const Reconstruction& self, const py::object nvm_path, bool skip_distortion) {
                std::string path = py::str(nvm_path).cast<std::string>();
                THROW_CHECK_HAS_FILE_EXTENSION(path, ".nvm");
                THROW_CHECK_FILE_OPEN(path);
                self.ExportNVM(path, skip_distortion);
            }, py::arg("output_path"), py::arg("skip_distortion") = false,
             "Export reconstruction in NVM format (.nvm).\n\n"
             "Only supports SIMPLE_RADIAL camera models when exporting\n"
             "distortion parameters. When skip_distortion == True it supports all camera\n"
             "models with the caveat that it's using the mean focal length which will be\n"
             "inaccurate for camera models with two focal lengths and distortion.")
        .def("export_CAM",
            [](const Reconstruction& self, const py::object cam_dir, bool skip_distortion) {
                std::string dir = py::str(cam_dir).cast<std::string>();
                THROW_CHECK_DIR_EXISTS(dir);
                self.ExportCam(dir, skip_distortion);
            }, py::arg("output_dir"), py::arg("skip_distortion") = false,
             "Exports in CAM format which is a simple text file that contains pose\n"
             "information and camera intrinsics for each image and exports one file per\n"
             "image; it does not include information on the 3D points. The format is as\n"
             "follows (2 lines of text with space separated numbers):\n"
             "<Tvec; 3 values> <Rotation matrix in row-major format; 9 values>\n"
             "<focal_length> <k1> <k2> 1.0 <principal point X> <principal point Y>\n"
             "Note that focal length is relative to the image max(width, height),\n"
             "and principal points x and y are relative to width and height respectively.\n\n"
             "Only supports SIMPLE_RADIAL and RADIAL camera models when exporting\n"
             "distortion parameters. When skip_distortion == True it supports all camera\n"
             "models with the caveat that it's using the mean focal length which will be\n"
             "inaccurate for camera models with two focal lengths and distortion.")
        .def("export_bundler",
            [](const Reconstruction& self,
               const py::object bundler_path, const py::object bundler_list_path,
               bool skip_distortion) {
                std::string path = py::str(bundler_path).cast<std::string>();
                std::string list_path = py::str(bundler_list_path).cast<std::string>();
                THROW_CHECK_HAS_FILE_EXTENSION(path, ".out");
                THROW_CHECK_HAS_FILE_EXTENSION(list_path, ".txt");
                THROW_CHECK_FILE_OPEN(path);
                THROW_CHECK_FILE_OPEN(list_path);
                self.ExportBundler(path, list_path, skip_distortion);
            }, py::arg("output_path"), py::arg("list_path"),
               py::arg("skip_distortion") = false,
             "Export reconstruction in Bundler format.\n"
             "Supports SIMPLE_PINHOLE, PINHOLE, SIMPLE_RADIAL and RADIAL camera models\n"
             "when exporting distortion parameters. When skip_distortion == True it\n"
             "supports all camera models with the caveat that it's using the mean focal\n"
             "length which will be inaccurate for camera models with two focal lengths\n"
             "and distortion.")
        .def(
            "export_PLY",
            [](const Reconstruction& self, const py::object ply_path) {
                std::string path = py::str(ply_path).cast<std::string>();
                THROW_CHECK_HAS_FILE_EXTENSION(path, ".ply");
                THROW_CHECK_FILE_OPEN(path);
                self.ExportPLY(path);
            }, py::arg("output_path"),
            "Export 3D points to PLY format (.ply).")
        .def(
            "export_VRML",
            [](const Reconstruction& self, const py::object images_path,
               const py::object points3D_path, const double image_scale,
               const Eigen::Vector3d& image_rgb) {
                std::string img_path = py::str(images_path).cast<std::string>();
                std::string p3D_path = py::str(points3D_path).cast<std::string>();
                THROW_CHECK_FILE_OPEN(img_path);
                THROW_CHECK_FILE_OPEN(p3D_path);
                THROW_CHECK_HAS_FILE_EXTENSION(img_path, ".wrl");
                THROW_CHECK_HAS_FILE_EXTENSION(p3D_path, ".wrl");
                self.ExportVRML(img_path, p3D_path, image_scale, image_rgb);
            },
            py::arg("images_path"),
            py::arg("points3D_path"),
            py::arg("image_scale") = 1.0,
            py::arg("image_rgb") = Eigen::Vector3d(1,0,0),
            "Export reconstruction in VRML format (.wrl).")
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
        .def(
            "check",
            [](colmap::Reconstruction& self) {
                for (auto& p3D_p : self.Points3D()) {
                    const colmap::Point3D& p3D = p3D_p.second;
                    const colmap::point3D_t p3Did = p3D_p.first;
                    for (auto& track_el : p3D.Track().Elements()) {
                        colmap::image_t image_id = track_el.image_id;
                        colmap::point2D_t point2D_idx = track_el.point2D_idx;
                        THROW_CHECK_MSG(self.ExistsImage(image_id), image_id);
                        THROW_CHECK_MSG(self.IsImageRegistered(image_id), image_id);
                        const colmap::Image& image = self.Image(image_id);
                        THROW_CHECK(image.IsRegistered());
                        THROW_CHECK_EQ(image.Point2D(point2D_idx).Point3DId(), p3Did)
                    }
                }
                for (auto& image_id : self.RegImageIds()) {
                    THROW_CHECK_MSG(self.Image(image_id).HasCamera(), image_id);
                    colmap::camera_t camera_id = self.Image(image_id).CameraId();
                    THROW_CHECK_MSG(self.ExistsCamera(camera_id), camera_id);
                }
            },
            "Check if current reconstruction is well formed.")
        .def("__copy__", [](const Reconstruction& self) { return Reconstruction(self); })
        .def("__deepcopy__",
             [](const Reconstruction& self, py::dict) { return Reconstruction(self); })
        .def("__repr__",
             [](const Reconstruction& self) {
                 std::stringstream ss;
                 ss << "<Reconstruction 'num_reg_images=" << self.NumRegImages()
                    << ", num_cameras=" << self.NumCameras()
                    << ", num_points3D=" << self.NumPoints3D()
                    << ", num_observations=" << self.ComputeNumObservations() << "'>";
                 return ss.str();
             })
        .def("summary", [](const Reconstruction& self) {
            std::stringstream ss;
            ss << "Reconstruction:"
               << "\n\tnum_reg_images = " << self.NumRegImages()
               << "\n\tnum_cameras = " << self.NumCameras()
               << "\n\tnum_points3D = " << self.NumPoints3D()
               << "\n\tnum_observations = " << self.ComputeNumObservations()
               << "\n\tmean_track_length = " << self.ComputeMeanTrackLength()
               << "\n\tmean_observations_per_image = " << self.ComputeMeanObservationsPerRegImage()
               << "\n\tmean_reprojection_error = " << self.ComputeMeanReprojectionError();
            return ss.str();
        });
}
