import numpy as np
from scipy import misc
from interpolate_2D import interpolate_2D

def rotate_z(points,theta):
    """Rotate an array of points theta degrees around the z-axis."""
    theta = np.radians(theta)
    c, s = np.cos(theta), np.sin(theta)
    rotationMatrix = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return np.dot(points,rotationMatrix)

def depth_to_point_cloud(imgHead, imgTail, imgLeft, imgRight, far, interpolate=False, threshold=1.0):
    """Returns a point cloud calculated from four depth map."""
    # Read depth images and decode them

    #edited
    head = decode_depthmap(imgHead, far)
    tail = decode_depthmap(imgTail, far)
    left = decode_depthmap(imgLeft, far)
    right = decode_depthmap(imgRight, far)

    # Get list of lidar ray angles and the coordinates in the image they intersect
    [coords, angles] = get_relevant_coordinates()
    x_coords, y_coords = coords[:,0], coords[:,1]

    n_points = len(x_coords)

    # Interpolate or get raw pixel values
    if interpolate:
        vHead = interpolate_2D(head, x_coords, y_coords, threshold)
        vTail = interpolate_2D(tail, x_coords, y_coords, threshold)
        vLeft = interpolate_2D(left, x_coords, y_coords, threshold)
        vRight = interpolate_2D(right, x_coords, y_coords, threshold)
    else:
        vHead = get_pixel_values(head, x_coords, y_coords)
        vTail = get_pixel_values(tail, x_coords, y_coords)
        vLeft = get_pixel_values(left, x_coords, y_coords)
        vRight = get_pixel_values(right, x_coords, y_coords)

    # Calculate 3D coordinates from ray angles and depth values
    cHead = get_coordinates(vHead,angles)
    cTail = get_coordinates(vTail,angles)
    cLeft = get_coordinates(vLeft,angles)
    cRight = get_coordinates(vRight,angles)

    # Rotate points according to the camera directions
    cTail = rotate_z(cTail,180)
    cLeft = rotate_z(cLeft,-90)
    cRight = rotate_z(cRight,90)

    # Concatenate points from all cameras and save as image
    pointCloud = np.concatenate((cHead,cTail,cLeft,cRight),0)
    return pointCloud

def get_pixel_values(values, x_coords, y_coords):
    x_coords = x_coords.astype(int)
    y_coords = np.ceil(y_coords).astype(int) - 1

    n_points = len(x_coords)
    pixel_values = np.zeros([n_points, 1])

    for i in range(n_points):
        pixel_values[i] = values[y_coords[i], x_coords[i]]

    return pixel_values

def get_relevant_coordinates():
    """Returns a numpy ndarray specifying the pixel a lidar ray hits when shot
    through the near plane."""
    coords_and_angles = np.genfromtxt('coords_and_angles.csv', delimiter=',')
    return np.hsplit(coords_and_angles,2)

def decode_depthmap_continuous(depth_map, far_plane_distance):
    """Decode CARLA-encoded depth values into meters."""
    depth_map = depth_map[:,:,0] + 256*depth_map[:,:,1] + (256*256)*depth_map[:,:,2];
    depth_map = depth_map / (256*256*256-1);
    depth_map = depth_map * far_plane_distance* 2;
    return depth_map

def spherical_to_cartesian(azimuth, elevation, radius):
    """Convert spherical coordinates into cartesian x,y and z coordinates"""
    x = radius * np.cos(elevation) * np.cos(azimuth)
    y = radius * np.cos(elevation) * np.sin(azimuth)
    z = radius * np.sin(elevation)
    return np.array([x,y,z])

def get_coordinates(values, angles):
    """Convert depth map values into corresponding lidar measurements."""
    coordinates = np.zeros((len(angles),3))

    # Correction for differences between lidar and depthmaps measurements
    correctionConstant = 1 / (np.cos(h) * np.cos(v))

    for i in range(len(angles)-1):
           v = np.radians(angles[i,0])
           h = np.radians(angles[i,1])
           r = values[i]


           r = r * correctionConstant

           # Calculate cartesian coordinates
           coordinates[i,:] = np.transpose(spherical_to_cartesian(h,v,r))

    return coordinates

if __name__ == "__main__":
    main()
