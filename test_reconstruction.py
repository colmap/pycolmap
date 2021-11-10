import pycolmap
import numpy as np


def test_tform(num_coords):
    orig_tform = pycolmap.SimilarityTransform3(
        2, [0.1, 0.3, 0.2, 0.4], [100, 10, 0.5])
    src = []
    dst = []

    for i in range(num_coords):
        src.append(np.array([i, i + 2.0, i * i]))
        dst.append(src[-1].copy())
        orig_tform.transform_point(dst[-1])

    est_tform = pycolmap.SimilarityTransform3.estimate(src, dst)
    assert(np.linalg.norm(orig_tform.matrix - est_tform.matrix) < 1e-6)


if __name__ == "__main__":
    test_tform(3)
    test_tform(100)
    # Create reconstruction instance
    reconstruction = pycolmap.Reconstruction()

    input_path = "input_reconstruction/"
    output_path = "output_reconstruction/"

    # Read a model
    reconstruction.read(input_path)

    # Print reconstruction statistics
    print(reconstruction)
    print(reconstruction.summary())

    # Accessing objects
    image_id = 1
    my_image = reconstruction.images[image_id]
    print(my_image)
    print(my_image.summary())

    #images, cameras and points3D of a reconstruction are passed-by-reference and can be edited in-place
    my_image.name = "my_image.png"
    assert(my_image.name == reconstruction.images[image_id].name)

    point3D_id = list(reconstruction.point3D_ids())[0]

    my_point = reconstruction.points3D[point3D_id]
    print(my_point)
    print(my_point.summary())
    world = my_point.xyz
    world[1] = 5
    assert(reconstruction.points3D[point3D_id].y == world[1])
    assert(my_point.xyz[1] == world[1])
    assert(my_point.xyz[1] == 5)
    my_point.x = 3
    assert(reconstruction.points3D[point3D_id].xyz[0] == world[0])
    assert(reconstruction.points3D[point3D_id].xyz[0] == my_point.x)
    print(my_point.track.elements)

    #primitve objects are NOT mutable.
    my_x = my_point.x
    my_x = 2.0
    assert(my_point.x != my_x)

    # To create copies, all objects support __copy__ and __deepcopy__
    my_image_copy = my_image.__copy__()
    my_image_copy.image_id = 10000
    assert(my_image_copy.image_id != my_image.image_id)

    # Write
    reconstruction.write(output_path) #writes binary
    reconstruction.write_text(output_path)
    reconstruction.export_PLY(output_path + "test.ply")

    # Constructors
    track = pycolmap.Track([pycolmap.TrackElement(2,1), pycolmap.TrackElement(3,5)])
    point3D = pycolmap.Point3D([1.0,2.2,-1.0], track)
    print(point3D.summary())

    img = pycolmap.Image("image1.jpg")
    img = pycolmap.Image("image2.jpg", keypoints = [[1.5,2.5], [3.0,4.0]], \
            tvec = [0.5,0.5,0.0], qvec = [1.0,0.0,0.1,0.0])

    cam = pycolmap.Camera("SIMPLE_PINHOLE", 1600, 1200, [3.0,800,600])

    # Transformations
    cam.image_to_world([1.0,0.5]) #1-point
    cam.image_to_world([[1.0,0.5], [2.0,1.0]])
    cam.image_to_world(img.points2D)
    cam.world_to_image([[110.0,120.0], [0.0,1.0]])

    # From World Coordinates to [u,v]
    cam.world_to_image(img.project([[1,2,3], [3,2,3]]))

    #world point
    xyz = [1,2,3]

    #2D points in image coordinates
    xy = img.project(xyz) #returns hnormalized points

    #pixel coordinates
    uv = cam.world_to_image(xy)

    #reprojection from pixels to image
    xy_repr = cam.image_to_world(uv)

    assert(np.linalg.norm(xy - xy_repr) < 1.0e-8)

    # homogeneous image coordinates
    xyz_homogeneous = np.hstack((xy_repr, [1.0]))

    depth = img.transform_to_image(xyz)[2] #returns points with depth

    #multiply homogeneous image points with depth for 3D points in image coordinates
    xyz_image = xyz_homogeneous * depth

    # transform back to world coordinates
    xyz_repr = img.transform_to_world(xyz_image)

    assert(np.linalg.norm(np.array(xyz).transpose() - xyz_repr) < 1.0e-8)