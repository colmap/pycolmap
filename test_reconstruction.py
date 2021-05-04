import pycolmap
import numpy as np
# Create reconstruction instance
reconstruction = pycolmap.Reconstruction()

input_path = "input_reconstruction/"
output_path = "output_reconstruction/"

# Read a model
reconstruction.read(input_path)

# Print reconstruction statistics
print("Reconstruction __repr__:") 
print(reconstruction)
print("Reconstruction summary():") 
print(reconstruction.summary())

# Accessing objects
image_id = 2
my_image = reconstruction.images[image_id]
print(my_image)
print(my_image.summary())

# Non-primite objects are all mutable, i.e. no copy
my_image.name = "my_image.png"
assert(my_image.name == reconstruction.images[image_id].name)
point3D_id = 10
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

#primitve objects are NOT mutable!!!!
my_x = my_point.x
my_x = 2.0
assert(my_point.x != my_x)

# To create copies, all objects support __copy__ and __deepcopy__
my_image_copy = my_image.__copy__()
my_image_copy.image_id = 10000
assert(my_image_copy.image_id != my_image.image_id)

# Write
#reconstruction.write(output_path)
#reconstruction.write_text(output_path)
#rec.export_PLY(output_path + "test.ply")

# Constructors
track = pycolmap.Track([pycolmap.TrackElement(2,1), pycolmap.TrackElement(3,5)])
point3D = pycolmap.Point3D([1.0,2.2,-1.0], track)
print(point3D.summary())

img = pycolmap.Image("def")
img = pycolmap.Image("abc", keypoints = [[1.5,2.5], [3.0,4.0]], tvec = [0.5,0.5,0.0], qvec = [1.0,0.0,0.1,0.0])
print(img.summary())

rec = pycolmap.Reconstruction(input_path)
print(rec.summary())


cam = pycolmap.Camera("SIMPLE_PINHOLE", 1600, 1200, [3.0,800,600])
print(cam.summary())

# Transformations
print(cam.image_to_world([1.0,0.5])) #1-point
print(cam.image_to_world([[1.0,0.5], [2.0,1.0]]))
print(cam.image_to_world(img.points2D))
print(cam.world_to_image([[110.0,120.0], [0.0,1.0]]))



