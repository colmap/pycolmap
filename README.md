# Python bindings for COLMAP estimators

At the moment, we provide bindings for essential and fundamental matrix estimation as well as absolute pose estimation.

# Getting started

Clone the repository and its submodules by running:
```
git clone --recursive git@github.com:mihaidusmanu/pycolmap.git
```

COLMAP should be installed as a library before proceeding. Please refer to the official website for installation instructions. PyCOLMAP can be installed using `pip`: 
```
pip install ./
```

# Usage

## Camera parameters

The current bindings are compatible with numpy arrays for both 2D and 3D points. The camera parameters should be sent as a Python dictionary with the following template:
```python
{
    'model': COLMAP_CAMERA_MODEL_NAME,
    'width': IMAGE_WIDTH,
    'height': IMAGE_HEIGHT,
    'params': EXTRA_CAMERA_PARAMETERS_LIST
}
```
Please refer to [colmap - src/base/camera_models.h](https://github.com/colmap/colmap/blob/master/src/base/camera_models.h) for more details regarding camera models and parameters.

## Absolute pose estimation

For instance, the following snippet runs absolute pose estimation for a pinhole camera given 2D-3D correspondences:
```python
import pycolmap

# Parameters:
# - points2D: Nx2 array; pixel coordinates
# - points3D: Nx3 array; world coordinates
# - camera_dict: dictionary
# Named parameters
# - max_error_px: float; RANSAC inlier threshold in pixels
answer = pycolmap.absolute_pose_estimation(
    points2D, points3D,
    {
        'model': 'SIMPLE_PINHOLE',
        'width': width,
        'height': height,
        'params': [focal_length, cx, cy]
    }
)
# Returns:
# - dictionary containing the RANSAC output
```

## SIFT feature extraction

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
- [ ] Expose more RANSAC parameters to Python
