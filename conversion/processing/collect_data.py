import numpy as np
from scipy import misc
from depth_to_point_cloud import depth_to_point_cloud
from point_cloud_to_image import trim_to_roi
from PIL import Image
import tqdm
import glob

ROI = 80        # Region of interest side length, in meters.
CELLS = 600     # Number of cells on a side in the output image
FAR = 1000      # Far plane distance
THRESHOLD = 1.5
name = 0



# Read depth maps
# head = misc.imread('../convert_image/head/{}.png'.format(name))
# tail = misc.imread('../convert_image/tail/{}.png'.format(name))
# left = misc.imread('../convert_image/left/{}.png'.format(name))
# right = misc.imread('../convert_image/right/{}.png'.format(name ))

heads = glob.glob("../convert_image/head/*")
tails = glob.glob("../convert_image/tail/*")
lefts = glob.glob("../convert_image/left/*")
rights = glob.glob("../convert_image/right/*")

for head, tail, left, right in zip(heads, tails, lefts, rights):

	name = head[len("../convert_image/head/"):-4]

	head = misc.imread(head)
	tail = misc.imread(tail)
	left = misc.imread(left)
	right = misc.imread(right)

	# Convert depth maps to 3D point cloud
	point_cloud = depth_to_point_cloud(head, tail, left, right, FAR, interpolate=True, threshold=THRESHOLD)
	# Trim point cloud to only contain points within the region of interest
	point_cloud = trim_to_roi(point_cloud,ROI)
	# TODO Count number of points within each grid cell
	# TODO Save as image

	grid = np.zeros([CELLS,CELLS])

	for point in point_cloud:
	    x, y, z = point
	    x += ROI/2
	    x /= ROI
	    cell_x = int(x*CELLS)

	    y += ROI/2
	    y /= ROI
	    cell_y = int(y*CELLS)

	    grid[cell_x,cell_y] += 1

	grid = 64*grid

	img = Image.fromarray(grid)
	img = img.rotate(180);
	img.convert('RGB').save("../convert_image/Pointcloud_images/{}.png".format(name))
	np.savetxt("../convert_image/Pointclouds/{}.bin".format(name),point_cloud.tolist())
	# img.show()

# np.savetxt("pointCloudForTestInMatLab.csv", point_cloud, delimiter=",")
# print(point_cloud)