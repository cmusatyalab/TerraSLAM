# TerraSLAM: Towards GPS-Denied Localization

This repository contains the **TerraSLAM** system, a comprehensive system support to transform SLAM localization results from **local**, **relative** coordinates to **global**, **geospatial** ones like GPS. 
If you find this work helpful, please consider citing:
```bibtex
@inproceedings{xu2025terraslam,
  author = {Xu, Jingao and Bala, Mihir and Eiszler, Thomas and Chen, Xiangliang and Dong, Qifei and Chanana, Aditya and Pillai, Padmanabhan and Satyanarayanan, Mahadev},
  booktitle = {Proceedings of the ACM MobiSys},
  title = {TerraSLAM: Towards GPS-Denied Localization},
  year = {2025},
}
```

The complete TerraSLAM package consists of three components:

1. [**ORB-SLAM3 ROS2 Interface Docker**](#1-setup-orb-slam3-ros2-interface-docker): Dockerized wrapper for ORB-SLAM3 on ROS 2 Humble for Ubuntu 22.04
2. [**TerraSLAM Relay**](#2-setup-terraslam-relay): Communication relay system for SLAM resluts transmission and GPS coordinates calculation
3. [**SLAM-GIS-GPS Alignment**](#3-setup-slam-gis-gps-alignment): Integration module for aligning SLAM results with GIS and GPS coordinates


## 1. Setup ORB-SLAM3 ROS2 Interface Docker
This part provides the dockerized comprehensive wrapper for ORB-SLAM3 on ROS 2 Humble for Ubuntu 22.04.
Currently, it supports both ``Monocular`` and ``Stereo`` setup for ORB-SLAM3.

### 1.1 Clone this repository

1. ```git clone https://github.com/cmusatyalab/TerraSLAM.git```
2. ```cd TerraSLAM```
3. ```git submodule update --init --recursive --remote```

### 1.2 Install Docker on your system

Skip this step if you have already installed docker

```bash
cd TerraSLAM
sudo chmod +x container_root/shell_scripts/docker_install.sh
./container_root/shell_scripts/docker_install.sh
```

### 1.3 Build the image with ORB_SLAM3

1. Build the image: ```sudo docker build -t orb-slam3-humble:22.04 .```
2. Add ```xhost +``` to your ```.bashrc``` to support correct x11-forwarding using ```echo "xhost +" >> ~/.bashrc```
3. ```source ~/.bashrc```
4. You can see the built images on your machine by running ```sudo docker images```.

### 1.4 Running the container

1. ```cd TerraSLAM``` (ignore if you are already in the folder)
2. ```sudo docker compose run orb_slam3_22_humble```
3. This should take you inside the container. Once you are inside, run the command ```xeyes``` and a pair of eyes should pop-up. If they do, x11 forwarding has correctly been setup on your computer.
4. Once you have constructed the container, you can further work into it through ```docker exec -it -e DISPLAY=$DISPLAY @container_id /bin/bash```

### 1.5 Building the ORB-SLAM3 Wrapper

Launch the container using steps in (4).
```bash
cd /root/colcon_ws/
colcon build --symlink-install
source install/setup.bash
```

### 1.6 Launching ORB-SLAM3

Launch the container using steps in (4).
If you are inside the container, run the following:

* Monocular: ```ros2 launch orb_slam3_ros2_wrapper unirobot_mono.launch.py```
* Stereo: ```ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py```

### [Optional]1.7 Running this with Olympe

1. Set up your Olympe development environment [here](https://developer.parrot.com/docs/olympe/installation.html). Using a virtual environment is highly recommanded [here](https://developer.parrot.com/docs/olympe/pip_on_debian_based_distros.html#best-practices).
2. Generate a parameter file of your Olympe parrot. We have prepared one if you want to use:
```bash
cd /root/colcon_ws/orb_slam3_ros2_wrapper/params
cp olympe.yaml.temp olympe.yaml
```
3. Change the ``*.yaml`` parameter file you want to use in ``unirobot_mono.launch.py``
4. Launch ORB-SLAM3: ```ros2 launch orb_slam3_ros2_wrapper unirobot_mono.launch.py```. You should see a window popup which is waiting for images. This is partially indicative of the setup correctly done.
5. Open another terminal and feed images to ORB-SLAM3 through Ros2. Make sure you are in the Olympe development environment
```bash
cd /root/olympe_dev
python ros2_streaming.py
```

Because the mono type assume the depth based on the computer version so if you find the orbslam3 warning of "not initialized" please shake your camera! 

### [Optional]1.8 Running this with a Gazebo Classic simulation

1. Setup the ORB-SLAM3 ROS2 Docker using the steps above. Once you do (1) step in the ```Launching ORB-SLAM3``` section, you should see a window popup which is waiting for images. This is partially indicative of the setup correctly done.
2. Setup the simulation by following the README [here](https://github.com/suchetanrs/scout-husky-gazebo-ros2)
3. Once you are able to teleop the robot, you should be able to run ORB-SLAM3 with both the containers (simulation and wrapper) running parallely.

### Potential issues you may face.
The simulation and the wrapper both have their ```ROS_DOMAIN_ID``` set to 55 so they are meant to work out of the box. However, you may face issues if this environment variable is not set properly. Before you start the wrapper, run ```ros2 topic list``` and make sure the topics namespaced with ```scout_2``` are visible inside the ORB-SLAM3 container provided the simulation is running along the side.

## 2. Setup TerraSLAM Relay
This component provides a communication relay system for SLAM results transmission and GPS coordinates calculation. It acts as a bridge between the SLAM system and the GPS alignment module.

### 2.1 Overview
The TerraSLAM Relay component is located in the `container_root/TerraSLAM_relay/` directory and includes:
- `relay.py` - Main relay server
- `relay_client.py` - Client interface for relay communication
- `relay_backdoor.py` - Backdoor access for debugging
- `transform_utils.py` - Utility functions for coordinate transformations

### 2.2 Usage
Please refer to the documentation in the `container_root/TerraSLAM_relay/` directory for detailed setup and usage instructions.

## 3. Setup SLAM-GIS-GPS Alignment
This component provides the integration module for aligning SLAM results with GIS and GPS coordinates, enabling the transformation from local SLAM coordinates to global geospatial coordinates.

### 3.1 Overview
The SLAM-GIS-GPS Alignment module handles the transformation of SLAM localization results from local, relative coordinates to global, geospatial coordinates compatible with GPS systems.

### 3.2 Usage
Please refer to the respective documentation for detailed setup and usage instructions for the GPS alignment functionality.
