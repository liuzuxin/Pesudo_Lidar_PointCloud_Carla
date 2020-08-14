import numpy as np
from scipy import misc
from depth_to_point_cloud import depth_to_point_cloud
from point_cloud_to_image import trim_to_roi
from PIL import Image
import tqdm
import glob
import shutil
import os
import sys
import json
import imageio
from tqdm import tqdm

ROI = 100        # Region of interest side length, in meters.
CELLS = 600     # Number of cells on a side in the output image
FAR = 1000      # Far plane distance
THRESHOLD = 1.5
name = 0
   
    
with open('../lidar_config.txt') as f:
    lidar_config = json.load(f)
    index = len(lidar_config['lidar'])

### RPM HORIZONTAL_RES
#   300    0.1°
#   600    0.2°
#   900    0.3°
#  1200    0.4°

for lidar in tqdm(range(index)):
    
    vertical = lidar_config['lidar'][lidar]['angles']
    HORIZONTAL_RES = lidar_config['lidar'][lidar]['resolution']
        
    #vertical = [-25,-1,-1.667,-15.639,-11.31,0,-0.667,-8.843,-7.254,0.333,-0.333,-6.148,-5.333,
    #            1.333,0.667,-4,-4.667,1.667,1,-3.667,-3.333,3.333,2.333,-2.667 ,-3,7,4.667,-2.333 ,-2,15,10.333,-1.333]
    #HORIZONTAL_RES = 0.4
    
    vertical.sort()
    
    ALTITUDE_RES = vertical
    
#    To remove previous files and prevent overlap
    try:
        files = [glob.glob('../convert_image/{}/Pointcloud_images/*'.format(lidar)),glob.glob('../convert_image/{}/Pointclouds/*'.format(lidar)),glob.glob('../convert_image/{}/Labels/*'.format(lidar)),
             glob.glob('../convert_image/{}/head/*'.format(lidar)),glob.glob('../convert_image/{}/left/*'.format(lidar)),glob.glob('../convert_image/{}/tail/*'.format(lidar)),
             glob.glob('../convert_image/{}/right/*'.format(lidar)),glob.glob('../convert_image/{}/lidar/*'.format(lidar))]
        for i in range(len(files)):
            for f in files[i]:
                os.remove(f)
    except OSError as e:
        print(e)
    
    images = [glob.glob('../{}_datacollected_[i]*/head/*'.format(lidar)),glob.glob('../{}_datacollected_[i]*/left/*'.format(lidar)),
              glob.glob('../{}_datacollected_[i]*/tail/*'.format(lidar)),glob.glob('../{}_datacollected_[i]*/right/*'.format(lidar)),glob.glob('../{}_datacollected_[i]*/lidar/*'.format(lidar))]
    angles = ['head/','left/','tail/','right/','lidar/']
    for i in tqdm(range(len(images))):
        for f in images[i]:
            save_dir = '../convert_image/{}/'.format(lidar) +angles[i]
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
            shutil.copy(f,save_dir)    
            
    
    heads = glob.glob("../convert_image/{}/head/*".format(lidar))
    tails = glob.glob("../convert_image/{}/tail/*".format(lidar))
    lefts = glob.glob("../convert_image/{}/left/*".format(lidar))
    rights = glob.glob("../convert_image/{}/right/*".format(lidar))
    
    for head, tail, left, right in tqdm(zip(heads, tails, lefts, rights)):
    
    	name = head[len("../convert_image/{}/head/".format(lidar)):-4]
    
    	head = imageio.imread(head)
    	tail = imageio.imread(tail)
    	left = imageio.imread(left)
    	right = imageio.imread(right)
    
    	# Convert depth maps to 3D point cloud
    	point_cloud = depth_to_point_cloud(head, tail, left, right, FAR,ALTITUDE_RES,HORIZONTAL_RES,interpolate=True, threshold=THRESHOLD)
    
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
    	img = img.rotate(180)
    	if not os.path.exists("../convert_image/{}/Pointcloud_images/".format(lidar)):
    		os.makedirs("../convert_image/{}/Pointcloud_images/".format(lidar))    
    	if not os.path.exists("../convert_image/{}/Pointclouds/".format(lidar)):
    		os.makedirs("../convert_image/{}/Pointclouds/".format(lidar))      
    	if not os.path.exists("../convert_image/{}/Labels/".format(lidar)):
    		os.makedirs("../convert_image/{}/Labels/".format(lidar))    
    	img.convert('RGB').save("../convert_image/{}/Pointcloud_images/{:06d}.png".format(lidar,int(name)))
    	np.savetxt("../convert_image/{}/Pointclouds/{:06d}.bin".format(lidar,int(name)),point_cloud.tolist())
    
    # Copying labels for processing 
    for label in tqdm(glob.glob('../{}_datacollected_[l]*/*'.format(lidar))):
        try:
            shutil.copy(label,'../convert_image/{}/Labels/'.format(lidar))
        except OSError:
            print("Could not save labels")
