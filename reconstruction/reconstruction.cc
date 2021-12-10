#include "colmap/base/reconstruction.h"
#include "colmap/util/ply.h"
#include "colmap/base/projection.h"
#include "colmap/util/misc.h"
#include "colmap/util/types.h"
#include "colmap/base/camera_models.h"

using namespace colmap;

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>

namespace py = pybind11;
using namespace pybind11::literals;

#include "log_exceptions.h"
#include "reconstruction/track.cc"
#include "reconstruction/point2D.cc"
#include "reconstruction/point3D.cc"
#include "reconstruction/image.cc"
#include "reconstruction/camera.cc"

template<typename... Args>
      using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;
    
void init_track(py::module &);
void init_point2D(py::module &);
void init_point3D(py::module &);
void init_image(py::module &);
void init_camera(py::module &);

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


template <bool kEstimateScale>
bool ComputeRobustAlignmentBetweenPoints(
        const std::vector<Eigen::Vector3d>& src,
        const std::vector<Eigen::Vector3d>& dst,
        double max_error, double min_inlier_ratio,
        Eigen::Matrix3x4d* alignment) {

    RANSACOptions ransac_options;
    ransac_options.max_error = max_error;
    ransac_options.min_inlier_ratio = min_inlier_ratio;

    LORANSAC<SimilarityTransformEstimator<3, kEstimateScale>,
            SimilarityTransformEstimator<3, kEstimateScale>>
        ransac(ransac_options);

    const auto report = ransac.Estimate(src, dst);

    if (report.success) {
        *alignment = report.model;
    }
    return report.success;
}

//Reconstruction Bindings
void init_reconstruction(py::module &m) {
    // STL Containers, required for fast looping over members (avoids copying)
    using ImageMap = EIGEN_STL_UMAP(colmap::image_t, colmap::Image);
    using Point3DMap = EIGEN_STL_UMAP(colmap::point3D_t, colmap::Point3D);
    using CameraMap = EIGEN_STL_UMAP(colmap::camera_t, colmap::Camera);

    init_track(m);
    init_point2D(m);
    init_point3D(m);
    init_image(m);
    init_camera(m);

    py::class_<Reconstruction>(m, "Reconstruction")
        .def(py::init<>())
        // .def_property_readonly_static("from_folder", [](py::object){
        //     return py::cpp_function([](py::object input_path){
        //         py::str py_path = py::str(input_path);
        //         std::string path = py_path.cast<std::string>();
        //         THROW_CUSTOM_CHECK_MSG(
        //             ExistsReconstruction(path),
        //             std::invalid_argument,
        //             (std::string("cameras, images, points3D not found at ")
        //                 +path).c_str()
        //         );
        //         auto reconstruction = std::unique_ptr<Reconstruction>(new Reconstruction());
        //         reconstruction->Read(path);
        //         return reconstruction;
        //     }, py::arg("sfm_dir"));
        // })
        .def(py::init([](const py::object input_path){
            py::str py_path = py::str(input_path);
            std::string path = py_path.cast<std::string>();
            THROW_CUSTOM_CHECK_MSG(
                ExistsReconstruction(path),
                std::invalid_argument,
                (std::string("cameras, images, points3D not found at ")
                    +path).c_str()
            );
            auto reconstruction = std::unique_ptr<Reconstruction>(new Reconstruction());
            reconstruction->Read(path);
            return reconstruction;
        }), py::arg("sfm_dir"))
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
        .def("num_cameras", &Reconstruction::NumCameras)
        .def("num_reg_images", &Reconstruction::NumRegImages)
        .def("num_points3D", &Reconstruction::NumPoints3D)
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
        .def("add_camera", 
            [](Reconstruction& self, const class colmap::Camera& camera) {
                THROW_CHECK(!self.ExistsCamera(camera.CameraId()));
                THROW_CHECK(camera.VerifyParams());
                self.AddCamera(camera);
            }, "Add new camera. There is only one camera per image, while multiple images\n"
                "might be taken by the same camera.")
        .def("add_image", 
            [](Reconstruction& self, const class colmap::Image& image, 
               bool check_not_registered) {
                THROW_CHECK(!self.ExistsImage(image.ImageId()));
                if (check_not_registered) {
                    THROW_CHECK(!image.IsRegistered());
                }
                self.AddImage(image);
                if (image.IsRegistered()){
                    THROW_CHECK_NE(image.ImageId(), colmap::kInvalidImageId);
                    self.Image(image.ImageId()).SetRegistered(false);  //Set true in next line
                    self.RegisterImage(image.ImageId());
                }
            }, py::arg("image"), py::arg("check_not_registered") = false,
                "Add new image. If Image.IsRegistered()==true, either throw "
                "if check_not_registered, or register image also in reconstr.")
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
        .def("register_image", [](colmap::Reconstruction& self, 
        colmap::image_t imid) {
            THROW_CHECK_EQ(self.Image(imid).IsRegistered(),self.IsImageRegistered(imid));
            self.RegisterImage(imid);
        }, "Register an existing image.")
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
        .def("transform", [](Reconstruction& self, const Eigen::Matrix3x4d& tform_mat) {
            SimilarityTransform3 tform(tform_mat);
            self.Transform(tform);
        }, "Apply the 3D similarity transformation to all images and points.")
        .def("transform", &Reconstruction::Transform, 
            "Apply the 3D similarity transformation to all images and points.")
        .def("merge", &Reconstruction::Merge,
                "Merge the given reconstruction into this reconstruction by registering the\n"
                "images registered in the given but not in this reconstruction and by\n"
                "merging the two clouds and their tracks. The coordinate frames of the two\n"
                "reconstructions are aligned using the projection centers of common\n"
                "registered images. Return true if the two reconstructions could be merged.")
        .def("align_poses", [](Reconstruction& self, const Reconstruction& ref_reconstruction,
             const double min_inlier_observations, double max_reproj_error){
                THROW_CHECK_GE(min_inlier_observations, 0.0);
                THROW_CHECK_LE(min_inlier_observations, 1.0);
                Eigen::Matrix3x4d alignment;
                bool success = 
                    ComputeAlignmentBetweenReconstructions(self,
                        ref_reconstruction, min_inlier_observations, 
                        max_reproj_error, &alignment);
                THROW_CHECK(success);
                SimilarityTransform3 tform(alignment);
                self.Transform(tform);
                return tform;
             }, py::arg("ref_reconstruction"), 
                py::arg("min_inlier_observations") = 0.3, 
                py::arg("max_reproj_error") = 8.0,
                "Align to reference reconstruction using RANSAC.")
        .def("align_points", [](Reconstruction& self, const Reconstruction& ref,
                int min_overlap, double max_error, double min_inlier_ratio){
            std::vector<Eigen::Vector3d> src;
            std::vector<Eigen::Vector3d> dst;
            // std::vector<std::map<point3D_t,size_t>> counts;
            for (auto& p3D_p : self.Points3D()) {
                std::map<point3D_t,size_t> counts;
                const Track& track = p3D_p.second.Track();
                for (auto& track_el : track.Elements()) {
                    if (!ref.IsImageRegistered(track_el.image_id)) {
                        continue;
                    }
                    const Point2D& p2D_dst = 
                    ref.Image(track_el.image_id).Point2D(track_el.point2D_idx);
                    if (p2D_dst.HasPoint3D()) {
                        if (counts.find(p2D_dst.Point3DId()) != counts.end()) {
                            counts[p2D_dst.Point3DId()]++;
                        } else {
                            counts[p2D_dst.Point3DId()] = 0;
                        }
                    }
                }
                if (counts.size() == 0) {
                    continue;
                }
                auto best_p3D = std::max_element(
                counts.begin(), counts.end(),
                []( const std::pair<point3D_t, size_t>& p1, 
                    const std::pair<point3D_t, size_t>& p2) 
                {return p1.second < p2.second; });
                if (best_p3D->second >= min_overlap) {
                    src.push_back(p3D_p.second.XYZ());
                    dst.push_back(ref.Point3D(best_p3D->first).XYZ());
                }
            }
            THROW_CHECK_EQ(src.size(), dst.size());
            std::cerr<<"Found "<<src.size()<< " / "<<self.NumPoints3D()
            <<" valid correspondences."<<std::endl;
            
            Eigen::Matrix3x4d alignment;
            bool success = 
                    ComputeRobustAlignmentBetweenPoints<true>(
                    src, dst, max_error, min_inlier_ratio, &alignment);
            THROW_CHECK(success);
            SimilarityTransform3 tform(alignment);
            self.Transform(tform);
            return tform;
        }, py::arg("reference_reconstruction"),
            py::arg("min_overlap") = 3,
            py::arg("max_error") = 0.005,
            py::arg("min_inlier_ratio") = 0.9,
            "Align 3D points to reference reconstruction using LORANSAC.\n"
            "Estimates pose by aligning corresponding 3D points.\n"
            "Correspondences are estimated by counting similar detection idxs.\n"
            "Assumes image_ids and point2D_idx overlap.")    
        .def("align", [](Reconstruction& self, const std::vector<std::string>& image_names,
             const std::vector<Eigen::Vector3d>& locations,
             const int min_common_images){
                SimilarityTransform3 tform;
                THROW_CHECK_GE(min_common_images, 3);
                THROW_CHECK_EQ(image_names.size(),locations.size());
                bool success = self.Align(image_names, locations, min_common_images, &tform);
                THROW_CHECK(success);
                return tform;
             }, "Align the given reconstruction with a set of pre-defined camera positions.\n"
                "Assuming that locations[i] gives the 3D coordinates of the center\n"
                "of projection of the image with name image_names[i].")
        .def("align_robust", [](Reconstruction& self, const std::vector<std::string>& image_names,
             const std::vector<Eigen::Vector3d>& locations,
             const int min_common_images, double max_error, double min_inlier_ratio){
                SimilarityTransform3 tform;
                THROW_CHECK_GE(min_common_images, 3);
                THROW_CHECK_EQ(image_names.size(),locations.size());
                RANSACOptions options;
                options.max_error = max_error;
                options.min_inlier_ratio = min_inlier_ratio;
                bool success = self.AlignRobust(image_names, locations, 
                                          min_common_images, options, &tform);
                THROW_CHECK(success);
                return tform;
             }, py::arg("image_names"), py::arg("locations"), 
                py::arg("min_common_images"), py::arg("max_error") = 12.0, 
                py::arg("min_inlier_ratio") = 0.1,
                "Robust alignment using RANSAC. \n"
                "Align the given reconstruction with a set of pre-defined camera positions.\n"
                "Assuming that locations[i] gives the 3D coordinates of the center\n"
                "of projection of the image with name image_names[i].")
        .def("compute_bounding_box", &Reconstruction::ComputeBoundingBox,
            py::arg("p0")=0.0, py::arg("p1")=1.0)
        .def("crop",&Reconstruction::Crop)
        // .def("align_robust", &Reconstruction::AlignRobust,
        //         "Robust alignment using RANSAC.")
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
        .def("import_PLY",overload_cast_<const std::string&>()(&Reconstruction::ImportPLY),
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
        .def("check", [](colmap::Reconstruction& self) {
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
                    THROW_CHECK_EQ(image.Point2D(point2D_idx).Point3DId(),p3Did)
                }
            }
            for (auto& image_id : self.RegImageIds()) {
                THROW_CHECK_MSG(self.Image(image_id).HasCamera(), image_id);
                colmap::camera_t camera_id = self.Image(image_id).CameraId();
                THROW_CHECK_MSG(self.ExistsCamera(camera_id), camera_id);
            }
        }, "Check if current reconstruction is well formed.")
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

    

    m.def("compare_reconstructions",
    [](const Reconstruction& reconstruction1,
        const Reconstruction& reconstruction2,
        const double min_inlier_observations, 
        const double max_reproj_error,
        bool verbose) {

        THROW_CHECK_GE(min_inlier_observations, 0.0);
        THROW_CHECK_LE(min_inlier_observations, 1.0);
        auto PrintComparisonSummary=[](std::stringstream& ss,
                                    std::vector<double>& rotation_errors,
                                    std::vector<double>& translation_errors,
                                    std::vector<double>& proj_center_errors) {
            auto PrintErrorStats = [](std::stringstream& ss, 
                                        std::vector<double>& vals) {
                const size_t len = vals.size();
                if (len == 0) {
                    ss << "Cannot extract error statistics from empty input" << std::endl;
                    return;
                }
                std::sort(vals.begin(), vals.end());
                ss << "Min:    " << vals.front() << std::endl;
                ss << "Max:    " << vals.back() << std::endl;
                ss << "Mean:   " << Mean(vals) << std::endl;
                ss << "Median: " << Median(vals) << std::endl;
                ss << "P90:    " << vals[size_t(0.9 * len)] << std::endl;
                ss << "P99:    " << vals[size_t(0.99 * len)] << std::endl;
            };
            ss << "# Image pose error summary" << std::endl;
            ss << std::endl << "Rotation angular errors (degrees)" << std::endl;
            PrintErrorStats(ss, rotation_errors);
            ss << std::endl << "Translation distance errors" << std::endl;
            PrintErrorStats(ss, translation_errors);
            ss << std::endl << "Projection center distance errors" << std::endl;
            PrintErrorStats(ss, proj_center_errors);
        };

        std::stringstream ss;
        ss << std::endl<<"Reconstruction 1"<<std::endl;
        ss << StringPrintf("Images: %d", reconstruction1.NumRegImages())
                    << std::endl;
        ss << StringPrintf("Points: %d", reconstruction1.NumPoints3D())
                    << std::endl;

        ss << std::endl<<"Reconstruction 2"<<std::endl;
        ss << StringPrintf("Images: %d", reconstruction2.NumRegImages())
                    << std::endl;
        ss << StringPrintf("Points: %d", reconstruction2.NumPoints3D())
                    << std::endl;
        if (verbose) {py::print(ss.str()); ss.str("");};

        ss << std::endl<<"Comparing reconstructed image poses"<<std::endl;
        const auto common_image_ids =
            reconstruction1.FindCommonRegImageIds(reconstruction2);
        ss << StringPrintf("Common images: %d", common_image_ids.size())
                    << std::endl;
        if (verbose) {py::print(ss.str()); ss.str("");};

        Eigen::Matrix3x4d alignment;
        if (!ComputeAlignmentBetweenReconstructions(reconstruction2, reconstruction1,
                                                    min_inlier_observations,
                                                    max_reproj_error, &alignment)) {
            THROW_EXCEPTION(std::runtime_error,"=> Reconstruction alignment failed.");
        }

        const SimilarityTransform3 tform(alignment);
        ss << "Computed alignment transform:" << std::endl
                    << tform.Matrix() << std::endl;
        if (verbose) {py::print(ss.str()); ss.str("");};

        const size_t num_images = common_image_ids.size();
        std::vector<double> rotation_errors(num_images, 0.0);
        std::vector<double> translation_errors(num_images, 0.0);
        std::vector<double> proj_center_errors(num_images, 0.0);
        for (size_t i = 0; i < num_images; ++i) {
            const image_t image_id = common_image_ids[i];
            Image image1 = reconstruction1.Image(image_id);  // copy!
            Image image2 = reconstruction2.Image(image_id);  // copy!
            tform.TransformPose(&image2.Qvec(), &image2.Tvec());

            const Eigen::Vector4d normalized_qvec1 = NormalizeQuaternion(image1.Qvec());
            const Eigen::Quaterniond quat1(normalized_qvec1(0), normalized_qvec1(1),
                                        normalized_qvec1(2), normalized_qvec1(3));
            const Eigen::Vector4d normalized_qvec2 = NormalizeQuaternion(image2.Qvec());
            const Eigen::Quaterniond quat2(normalized_qvec2(0), normalized_qvec2(1),
                                        normalized_qvec2(2), normalized_qvec2(3));

            rotation_errors[i] = RadToDeg(quat1.angularDistance(quat2));
            translation_errors[i] = (image1.Tvec() - image2.Tvec()).norm();
            proj_center_errors[i] =
                (image1.ProjectionCenter() - image2.ProjectionCenter()).norm();
        }
        PrintComparisonSummary(ss, rotation_errors, translation_errors,
                            proj_center_errors);
        if (verbose) {py::print(ss.str()); ss.str("");};
        py::dict res(
                    "alignment"_a=alignment,
                    "rotation_errors"_a=rotation_errors,
                    "translation_errors"_a=translation_errors,
                    "proj_center_errors"_a=proj_center_errors);
        return res;
    },
    py::arg("src_reconstruction").noconvert(),
    py::arg("ref_reconstruction").noconvert(),
    py::arg("min_inlier_observations") = 0.3,
    py::arg("max_reproj_error") = 8.0,
    py::arg("verbose") = true,
    py::keep_alive<1,2>(),
    py::keep_alive<1,3>());
}