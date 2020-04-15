# Python bindings for COLMAP estimators

At the moment, we provide bindings for essential and fundamental matrix estimation as well as absolute pose estimation.

# Getting started

COLMAP should be installed as a library before proceeding. Please refer to the official website for installation instructions. PyCOLMAP can by installed using `pip`: 
```
pip install ./
```

# Usage

The current bindings are compatible with numpy arrays for both 2D and 3D points. The camera parameters should be sent as a Python dictionary with the following template:
```python
{
    'model': COLMAP_CAMERA_MODEL_NAME,
    'width': IMAGE_WIDTH,
    'height': IMAGE_HEIGHT,
    'params': EXTRA_CAMERA_PARAMETERS_LIST
}
```

For instance, the following snippet runs absolute pose estimation for a pinhole camera given 2D-3D correspondences:
```python
import pycolmap

# points2D - Nx2 array with pixel coordinates
# points3D - Nx3 array with world coordinates
# inlier_threshold - RANSAC inlier threshold in pixels
# answer - dictionary containing the RANSAC output
answer = pycolmap.absolute_pose_estimation(
    points2D, points3D,
    {
        'model': 'SIMPLE_PINHOLE',
        'width': width,
        'height': height,
        'params': [focal_length, cx, cy]
    },
    inlier_threshold
)
```

# TODO

- [ ] Add documentation
- [ ] Add more detailed examples
- [ ] Expose more RANSAC parameters to Python
