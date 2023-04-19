# Python bindings for COLMAP

This repository exposes to Python most [COLMAP](https://colmap.github.io/) reconstruction objects, such as Cameras and Points3D, as well as estimators for absolute and relative poses.

## Getting started

Wheels for Linux (Python 3.6/7/8/9) and macOS 10 & 11 (Python 3.7/8/9) can be install using pip:
```bash
pip install pycolmap
```

The wheels are automatically built and pushed to [pypi](https://pypi.org/project/pycolmap/) at each release.

### Building from source

Alternatively, we explain below how to compile PyCOLMAP from source.

COLMAP should first be installed as a library following [the official guide](https://colmap.github.io/install.html).
**We recommend always building PyCOLMAP against the latest commit of the COLMAP [`dev` branch](https://github.com/colmap/colmap/tree/dev) - using a previous COLMAP build might not work.**
Currently, the earliest supported COLMAP commit is [colmap/colmap@33e2692](https://github.com/colmap/colmap/commit/33e26924f950172751f044a89b964129b3728db8).

Then clone the repository and its submodules:

```
git clone --recursive git@github.com:colmap/pycolmap.git
```

And finally build PyCOLMAP:
```bash
cd pycolmap
pip install .
```

### Windows

*Building on Windows is currently not supported - please do not open any issues regarding this.*
Any contributions in this direction are welcome - please refer to issue [#34](https://github.com/colmap/pycolmap/issues/34).
As a temporary workaround, we suggest using [WSL](https://docs.microsoft.com/en-us/windows/wsl/install). 

##

## Reconstruction pipeline

PyCOLMAP provides bindings for multiple steps of the standard reconstruction pipeline. They are defined in `pipeline/` and include:

- extracting and matching SIFT features
- importing an image folder into a COLMAP database
- inferring the camera parameters from the EXIF metadata of an image file
- running two-view geometric verification of matches on a COLMAP database
- triangulating points into an existing COLMAP model
- running incremental reconstruction from a COLMAP database
- dense reconstruction with multi-view stereo

Sparse & Dense reconstruction from a folder of images can be performed with:
```python
output_path: pathlib.Path
image_dir: pathlib.Path

output_path.mkdir()
mvs_path = output_path / "mvs"
database_path = output_path / "database.db"

pycolmap.extract_features(database_path, image_dir)
pycolmap.match_exhaustive(database_path)
maps = pycolmap.incremental_mapping(database_path, image_dir, output_path)
maps[0].write(output_path)
# dense reconstruction
pycolmap.undistort_images(mvs_path, output_path, image_dir)
pycolmap.patch_match_stereo(mvs_path)  # requires compilation with CUDA
pycolmap.stereo_fusion(mvs_path / "dense.ply", mvs_path)
```

PyCOLMAP can leverage the GPU for feature extraction, matching, and multi-view stereo if COLMAP was compiled with CUDA support.

Similarly, PyCOLMAP can run Delauney Triangulation if COLMAP was compiled with CGAL support.
This requires to build the package from source and is not available with the PyPI wheels.

All of the above steps are easily configurable with python dicts which are recursively merged into
their respective defaults, e.g.

```python
pycolmap.extract_features(database_path, image_dir, sift_options={"max_num_features": 512})
# equivalent to
ops = pycolmap.SiftExtractionOptions()
ops.max_num_features = 512
pycolmap.extract_features(database_path, image_dir, sift_options=ops)
```

To list available options, use

```python
help(pycolmap.SiftExtractionOptions)
```

The default parameters can be looked up with

```python
print(pycolmap.SiftExtractionOptions().summary())
# or
print(pycolmap.SiftExtractionOptions().todict())
```


For another example of usage, see [`hloc/reconstruction.py`](https://github.com/cvg/Hierarchical-Localization/blob/master/hloc/reconstruction.py).

## Reconstruction object

We can load and manipulate an existing COLMAP 3D reconstruction:

```python
import pycolmap
reconstruction = pycolmap.Reconstruction("path/to/my/reconstruction/")
print(reconstruction.summary())

for image_id, image in reconstruction.images.items():
    print(image_id, image)

for point3D_id, point3D in reconstruction.points3D.items():
    print(point3D_id, point3D)

for camera_id, camera in reconstruction.cameras.items():
    print(camera_id, camera)

reconstruction.write("path/to/new/reconstruction/")
```

The object API mirrors the COLMAP C++ library. The bindings support many other operations, for example:

- projecting a 3D point into an image with arbitrary camera model:

```python
uv = camera.world_to_image(image.project(point3D.xyz))
```

- aligning two 3D reconstructions by their camera poses:

```python
tfm = reconstruction1.align_poses(reconstruction2)  # transforms reconstruction1 in-place
print(tfm.rotation, tfm.translation)
```


`pycolmap` also allows exporting models to TXT/NVM//CAM/Bundler/VRML/PLY (also supports `pathlib.Path` inputs).
`skip_distortion == True` enables exporting more camera models, with the caveat of
averaging the focal length parameters.

```python
# Exports reconstruction to COLMAP text format.
reconstruction.write_text("path/to/new/reconstruction/")

# Exports in NVM format http://ccwu.me/vsfm/doc.html#nvm.
reconstruction.export_NVM("rec.nvm", skip_distortion=False)

# Creates a <img_name>.cam file for each image with pose/intrinsics information.
reconstruction.export_CAM("image_dir/", skip_distortion=False)

# Exports in Bundler format https://www.cs.cornell.edu/~snavely/bundler/.
reconstruction.export_bundler("rec.bundler.out", "rec.list.txt", skip_distortion=False)

# exports 3D points to PLY format.
reconstruction.export_PLY("rec.ply")

# Exports in VRML format https://en.wikipedia.org/wiki/VRML.
reconstruction.export_VRML("rec.images.wrl", "rec.points3D.wrl",
                           image_scale=1.0, image_rgb=[1.0, 0.0, 0.0])
```


## Estimators

We provide robust RANSAC-based estimators for absolute camera pose (single-camera and multi-camera-rig), essential matrix, fundamental matrix, homography, and two-view relative pose for calibrated cameras.

All RANSAC and estimation parameters are exposed as objects that behave similarly as Python dataclasses. The RANSAC options are described in [`colmap/src/optim/ransac.h`](https://github.com/colmap/colmap/blob/dev/src/optim/ransac.h#L47-L76) and their default values are:

```python
ransac_options = pycolmap.RANSACOptions(
    max_error=4.0,  # reprojection error in pixels
    min_inlier_ratio=0.01,
    confidence=0.9999,
    min_num_trials=1000,
    max_num_trials=100000,
)
```

### Absolute pose estimation

For instance, to estimate the absolute pose of a query camera given 2D-3D correspondences:
```python
# Parameters:
# - points2D: Nx2 array; pixel coordinates
# - points3D: Nx3 array; world coordinates
# - camera: pycolmap.Camera
# Optional parameters:
# - max_error_px: float; RANSAC inlier threshold in pixels (default=12.0)
# - estimation_options: dict or pycolmap.AbsolutePoseEstimationOptions
# - refinement_options: dict or pycolmap.AbsolutePoseRefinementOptions
answer = pycolmap.absolute_pose_estimation(points2D, points3D, camera, max_error_px=12.0)
# Returns: dictionary of estimation outputs
```

2D and 3D points are passed as Numpy arrays or lists. The options are defined in [`estimators/absolute_pose.cc`](./estimators/absolute_pose.cc#L187-L220) and can be passed as regular (nested) Python dictionaries:

```python
pycolmap.absolute_pose_estimation(
    points2D, points3D, camera,
    estimation_options={'ransac': {'max_error': 12.0}},
    refinement_options={'refine_focal_length': True},
)
```

### Absolute Pose Refinement

```python
# Parameters:
# - tvec: List of 3 floats, translation component of the pose (world to camera)
# - qvec: List of 4 floats, quaternion component of the pose (world to camera)
# - points2D: Nx2 array; pixel coordinates
# - points3D: Nx3 array; world coordinates
# - inlier_mask: array of N bool; inlier_mask[i] is true if correpondence i is an inlier
# - camera: pycolmap.Camera
# Optional parameters:
# - refinement_options: dict or pycolmap.AbsolutePoseRefinementOptions
answer = pycolmap.pose_refinement(tvec, qvec, points2D, points3D, inlier_mask, camera)
# Returns: dictionary of refinement outputs
```

### Essential matrix estimation

```python
# Parameters:
# - points2D1: Nx2 array; pixel coordinates in image 1
# - points2D2: Nx2 array; pixel coordinates in image 2
# - camera1: pycolmap.Camera of image 1
# - camera2: pycolmap.Camera of image 2
# Optional parameters:
# - max_error_px: float; RANSAC inlier threshold in pixels (default=4.0)
# - options: dict or pycolmap.RANSACOptions
answer = pycolmap.essential_matrix_estimation(points2D1, points2D2, camera1, camera2)
# Returns: dictionary of estimation outputs
```

### Fundamental matrix estimation

```python
answer = pycolmap.fundamental_matrix_estimation(
    points2D1,
    points2D2,
    [max_error_px],  # optional RANSAC inlier threshold in pixels
    [options],       # optional dict or pycolmap.RANSACOptions
)
```

### Homography estimation

```python
answer = pycolmap.homography_matrix_estimation(
    points2D1,
    points2D2,
    [max_error_px],  # optional RANSAC inlier threshold in pixels
    [options],       # optional dict or pycolmap.RANSACOptions
)
```

### Two-view geometry estimation

COLMAP can also estimate a relative pose between two calibrated cameras by estimating both E and H and accounting for the degeneracies of each model.

```python
# Parameters:
# - points2D1: Nx2 array; pixel coordinates in image 1
# - points2D2: Nx2 array; pixel coordinates in image 2
# - camera1: pycolmap.Camera of image 1
# - camera2: pycolmap.Camera of image 2
# Optional parameters:
# - max_error_px: float; RANSAC inlier threshold in pixels (default=4.0)
# - options: dict or pycolmap.TwoViewGeometryOptions
answer = pycolmap.homography_matrix_estimation(points2D1, points2D2)
# Returns: dictionary of estimation outputs
```

 The options are defined in [`estimators/two_view_geometry.cc`](estimators/two_view_geometry.cc#L102-L117) and control how each model is selected. The return dictionary contains the relative pose, inlier mask, as well as the type of camera configuration, such as degenerate or planar. This type is an instance of the enum `pycolmap.TwoViewGeometry` whose values are explained in [`colmap/src/estimators/two_view_geometry.h`](https://github.com/colmap/colmap/blob/dev/src/estimators/two_view_geometry.h#L47-L67).

### Camera argument

All estimators expect a COLMAP camera object, which can be created as follow:

```python
camera = pycolmap.Camera(
    COLMAP_CAMERA_MODEL_NAME,
    IMAGE_WIDTH, 
    IMAGE_HEIGHT,
    EXTRA_CAMERA_PARAMETERS,
)
```

The different camera models and their extra parameters are defined in [`colmap/src/base/camera_models.h`](https://github.com/colmap/colmap/blob/master/src/base/camera_models.h). For example for a pinhole camera:

```python
camera = pycolmap.Camera(
    model='SIMPLE_PINHOLE',
    width=width,
    height=height,
    params=[focal_length, cx, cy],
)
```

Alternatively, we can also pass a camera dictionary:

```python
camera_dict = {
    'model': COLMAP_CAMERA_MODEL_NAME,
    'width': IMAGE_WIDTH,
    'height': IMAGE_HEIGHT,
    'params': EXTRA_CAMERA_PARAMETERS_LIST
}
```


## SIFT feature extraction

```python
import numpy as np
import pycolmap
from PIL import Image, ImageOps

# Input should be grayscale image with range [0, 1].
img = Image.open('image.jpg').convert('RGB')
img = ImageOps.grayscale(img)
img = np.array(img).astype(np.float) / 255.

# Optional parameters:
# - options: dict or pycolmap.SiftExtractionOptions
# - device: default pycolmap.Device.auto uses the GPU if available
sift = pycolmap.Sift()

# Parameters:
# - image: HxW float array
keypoints, scores, descriptors = sift.extract(img)
# Returns:
# - keypoints: Nx4 array; format: x (j), y (i), sigma, angle
# - scores: N array; DoG scores
# - descriptors: Nx128 array; L2-normalized descriptors
```

## TODO

- [ ] Add documentation
- [ ] Add more detailed examples
- [ ] Add unit tests for reconstruction bindings

Created and maintained by [Mihai Dusmanu](https://github.com/mihaidusmanu/), [Philipp Lindenberger](https://github.com/Phil26AT), [John Lambert](https://github.com/johnwlambert), [Paul-Edouard Sarlin](https://github.com/Skydes), and other contributors.
