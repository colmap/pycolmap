# Python bindings for COLMAP

This repository exposes to Python most [COLMAP](https://colmap.github.io/) reconstruction objects, such as Cameras and Points3D, as well as estimators for absolute and relative poses.

## Getting started

Wheels for Linux (Python 3.7 & 3.8) and macOS 10 & 11 (Python 3.8 & 3.9) can be install using pip:
```bash
pip install pycolmap
```

The wheels are automatically built and pushed to [pypi](https://pypi.org/project/pycolmap/0.0.1/) at each release.

### Building from source

Alternatively, we explain below how to compile PyCOLMAP from source. COLMAP should first be installed as a library following [the instructions](https://colmap.github.io/install.html). We require the latest commit of the COLMAP [`dev` branch](https://github.com/colmap/colmap/tree/dev). Using a previous COLMAP build might not work. Then clone the repository and its submodules:

```
git clone --recursive git@github.com:mihaidusmanu/pycolmap.git
```

And finally build PyCOLMAP:
```bash
cd pycolmap
pip install pycolmap
```

### Windows

On Windows, we recommend to install COLMAP with [vcpkg](https://github.com/microsoft/vcpkg). From your vcpkg directory, run:
```
.\vcpkg.exe install colmap --triplet=x64-windows --head
```

Then set the `CMAKE_TOOLCHAIN_FILE` environment variable to your `vcpkg\scripts\buildsystems\vcpkg.cmake` path. For Example in powershell:
```
$env:CMAKE_TOOLCHAIN_FILE='C:\Workspace\vcpkg\scripts\buildsystems\vcpkg.cmake'
```

Finally go to the PyCOLMAP folder and run
```
py -m pip install ./
```

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
reconstruction.export_PLY("path/to/new/reconstruction/reconstruction.ply")
```

The object API mirrors the COLMAP C++ library. The bindings support many other operations, for example:

- projecting a 3D point into an image with arbitrary camera model:

```python
uv = camera.world_to_image(image.project(point3D.xyz))
```

- aligning two 3D reconstruction by their camera poses:

```python
tfm = reconstruction1.align_poses(reconstruction2)  # transforms reconstruction1 in-place
print(tfm.rotation, tfm.translation)
```

## Estimators

### Absolute pose estimation

For instance, the following snippet estimates the absolute pose for a query camera given 2D-3D correspondences:
```python
# Parameters:
# - points2D: Nx2 array; pixel coordinates
# - points3D: Nx3 array; world coordinates
# - camera: pycolmap.Camera
# Named parameters
# - max_error_px: float; RANSAC inlier threshold in pixels
answer = pycolmap.absolute_pose_estimation(points2D, points3D, camera)
# Returns:
# - dictionary containing the RANSAC output
```

2D and 3D points can be passed as Numpy arrays.

### Absolute Pose Refinement

```python
# Parameters:
# - tvec: List of 3 floats, translation component of the pose (world to camera)
# - qvec: List of 4 floats, quaternion component of the pose (world to camera)
# - points2D: Nx2 array; pixel coordinates
# - points3D: Nx3 array; world coordinates
# - inlier_mask: array of N bool; inlier_mask[i] is true if correpondence i is an inlier
# - camera_dict: pycolmap.Camera
answer = pycolmap.pose_refinement(tvec, qvec, points2D, points3D, inlier_mask, camera)
# Returns:
# - dictionary containing the RANSAC output
```

### Essential matrix estimation

```python
return_dict = pycolmap.essential_matrix_estimation(
    points2D_1,  # 2D pixel coordinates in image 1
    points2D_2,  # 2D pixel coordinates in image 2
    camera_1,    # camera model of image 1
    camera_2,    # camera model of image 2
)
```

### Camera object

All estimators expect a COLMAP camera object, which can be created as follow:

```python
camera = pycolmap.Camera(
    COLMAP_CAMERA_MODEL_NAME,
    IMAGE_WIDTH, 
    IMAGE_HEIGHT,
    EXTRA_CAMERA_PARAMETERS,
)
```

The different camera models and their extra parameters are defined in [colmap/src/base/camera_models.h](https://github.com/colmap/colmap/blob/master/src/base/camera_models.h). For example for a pinhole camera:

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

### SIFT feature extraction

```python
import numpy as np
import pycolmap
from PIL import Image, ImageOps

# Input should be grayscale image with range [0, 1].
with open('image.jpg', 'rb') as f:
    img = Image.open(f)
    img = img.convert('RGB')
    img = ImageOps.grayscale(img)
    img = np.array(img).astype(np.float) / 255.

# Parameters:
# - image: HxW float array
# Named parameters:
# - num_octaves: int (4)
# - octave_resolution: int (3)
# - first_octave: int (0)
# - edge_thresh: float (10)
# - peak_thresh: float (0.01)
# - upright: bool (False)
keypoints, scores, descriptors = pycolmap.extract_sift(img)
# Returns:
# - keypoints: Nx4 array; format: x (j), y (i), sigma, angle
# - scores: N array; DoG scores
# - descriptors: Nx128 array; L2-normalized descriptors
```

# TODO

- [ ] Add documentation
- [ ] Add more detailed examples
- [ ] Add unit tests for reconstruction bindings
