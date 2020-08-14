# Data collection and parsing scripts
Goal: Simulate precise LiDAR point cloud data from Carla

### Starting sever:
Start CARLA server by running carlaUE4.sh from the compiled compressed file from the CARLA repository releases.
Check compatability of GPU to run the simulator and to vizualize the rendering.
Command to run the server:
```bash
SDL_VIDEODRIVER=offscreen SDL_HINT_CUDA_DEVICE=0 ./CarlaUE4.sh -carla-server -benchmark -fps=10
```
Or, use docker extensions to directly run the server.
Docker Installation:
!!! note Docker requires sudo to run. Follow this guide to add users to the docker sudo group https://docs.docker.com/install/linux/linux-postinstall/

Docker CE
https://docs.docker.com/install/linux/docker-ce/ubuntu/#extra-steps-for-aufs

NVIDIA-Docker2
To install nvidia-docker-2 we recommend using the "Quick Start" section from the nvidia-dockers https://github.com/NVIDIA/nvidia-docker

Commands:
```bash
docker pull carlasim/carla:0.9.4
docker run -p 2000-2002:2000-2002 --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=0 carlasim/carla:0.9.4
```
Edit the above commands to run specific version of Carla Server.

### Datacollection:
2 Terminals are rquired to collect data.

* 1st terminal:
```python
python spawn_npc.py -n 30
```
This command will spawn '30' number of vehicles of random types over random position of specified map on CARLA server. In this case it is default "Town03".


* 2nd terminal:
```python 
python lidar_depth_bboxes.py
```
This command will open a pygame window showing vizualization of the vehicle in 3rd person view with the bounding boxes rendered. And along with that, a part of the window surface will be vizualization of "logarithmic" depth in greyscale of the depth sensor.
At the same time, the data of the moment will be saved. The lidar pointcloud ".bin" file, if activated, will be saved in the folder "datacollected_^" where the ^ is the timestamp of start of data collection. The bounding box labels will be stored as a npy file in KITTI format (8 points of (x,y,z) of bounding box corners) in the folder "datacollected_labels_^". The dept images will be saved in folder "datacollected_images_^". This folder has subdirectory "head", "tail", "left" and "right" indicating depth images from all the 4 depth cameras.


### Processing:
To start processing the depth images to lidar pointcloud, transfered the required 4 camera images into the 4 folders in "convert_image" in the corresponding sequence of data collection frames named in whole numbers 0,1,2,... .
The folder "processing" contains script "main.py". Run the script to start processing depth images to accurate pointcloud from the smae directory.
The pointcloud will be stored in folder "Pointclouds" as bin file under "convert_image". And corresponding bird-eye views of pointclouds will be stored in folder "Pointcloud_images". The corresponding BBox labels will be transfered to "Labels" folder.
All the hyperparameters indicating type of lidar can be tuned in the script.


## Directory structure
```
conversion
├── convert_image
│   ├── Pointcloud_images
│   ├── Pointclouds
│   ├── Labels
│   ├── head
│   ├── right
│   ├── tail
│   ├── left
├── processing
│   ├── main.py
│   ├── point_cloud_to_image.py
│   ├── depth_to_point_cloud.py
│   ├── interpolate_2D.py
├── datacollected_*
├── datacollected_labels_*
├── datacollected_images_*
├── lidar_depth_bboxes.py
├── spawn_npc.py
├── manual_control.py
├── synchronous_mode.py
├── ... (Other files and folders)
```
