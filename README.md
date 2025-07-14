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

## Demo
![TerraSLAM Demo](terraslam_demo.gif)

[Full Video](https://youtu.be/kR1oD3MYbz0)

## Setup Overview
To run TerraSLAM and reproduce the demo above, you need to launch three modules:

1. [**TerraSLAM Docker with ORB-SLAM3 ROS2 Interface**](#1-setup-terraslam-docker): Dockerized TerraSLAM with wrapper for ORB-SLAM3 on ROS 2 Humble (Ubuntu 22.04). The ORB-SLAM3 wrapper is partially based on [suchetanrs's work](https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker.git)
2. [**TerraSLAM Relay**](#2-launch-orb-slam3-and-run-terraslam-relay): 
Calculate the transformation matrix among the SLAM, GIS, and GPS coordinate systems, and use it to convert SLAM localization results into GPS coordinates (latitude, longitude, altitude)
3. [**TerraSLAM Visualization**](#3-terraslam-visualization): 
Display the localization results on Google Maps (right part), and use [Blender 2.93](https://www.blender.org/download/lts/2-93/) to render the localization results within the GIS model (left side)


## 1. Setup TerraSLAM Docker 
TerraSLAM provides the dockerized wrapper for ORB-SLAM3 on ROS 2 Humble for Ubuntu 22.04.
Currently, it supports both ``Monocular`` and ``Stereo`` setup for ORB-SLAM3.

### Clone this repository

```bash
git clone https://github.com/cmusatyalab/TerraSLAM.git
cd TerraSLAM
git submodule update --init --recursive --remote
```


### Install Docker on your system

Skip this step if you have already installed docker

```bash
cd TerraSLAM
sudo chmod +x container_root/shell_scripts/docker_install.sh
./container_root/shell_scripts/docker_install.sh
```

### Build the image with ORB_SLAM3

* Build the image: 
    ```
    sudo docker build -t orb-slam3-humble:22.04 .
    ```
* You can see the built images on your machine by running 
    ```
    sudo docker images | grep orb-slam3-humble
    ```
* (Optional) Add ```xhost +``` to your ```.bashrc``` to support correct x11-forwarding using 
    ```
    echo "xhost +" >> ~/.bashrc
    source ~/.bashrc
    ```

### Run the container
* Run TerraSLAM container
    ```
    cd TerraSLAM
    sudo docker compose run TerraSLAM
    ```
* This should take you inside the container. Once you are inside, run the command ```xeyes``` and a pair of eyes should pop-up. If they do, x11 forwarding has correctly been setup on your computer.
* Once you have constructed the container, you can further work into it through: 
    ```
    docker exec -it -e DISPLAY=$DISPLAY TerraSLAM /bin/bash
    ```
    or
    ```
    docker exec -it -e DISPLAY=$DISPLAY @container_id /bin/bash
    ```
* In the constructed container, please firstly setup bash environments by 
    ``` 
    cd && mv bashrc_temp .bashrc
    source .bashrc
    ``` 

### Build the ORB-SLAM3 Wrapper

Launch the container and then:
```bash
cd /root/colcon_ws/
colcon build --symlink-install
source install/setup.bash
```

## 2. Launch ORB-SLAM3 and Run TerraSLAM Relay
Here we use the above demo as an example to show how to run TerraSLAM.

### Prepare the Data
* **Video Frames**: in the TerraSLAM container, download the compressed drone-captured video frame folder, [Mill-video](https://storage.cmusatyalab.org/terra-slam/mill-video.tgz), uncompress it, and move it into the ``Database`` folder. Also, copy the ``image_publish.py`` from the ``TerraSLAM_runtime`` folder to the ``Database`` folder.
  ```
  cd &&  mkdir -p Database
  cd Database
  wget https://storage.cmusatyalab.org/terra-slam/mill-video.tgz
  tar xzvf mill-video.tgz
  cp TerraSLAM_runtime/image_publish.py Database/image_publish.py
  ``` 

* **Pre-built SLAM Map**: in the TerraSLAM container, download the SLAM map, [Mill-19-Map](https://storage.cmusatyalab.org/terra-slam/Map-Mill-19-2024.osa) into the ``Map`` folder.
    ```
    cd &&  mkdir -p Map
    cd Map
    wget https://storage.cmusatyalab.org/terra-slam/Map-Mill-19-2024.osa
    ```
  Please refer to the ``ORB_SLAM3`` submodule's README for instructions on how to save, import, and merge maps created by SLAM.
* **SLAM-GPS Transformation Matrix**: in the TerraSLAM container, download the Mill-19 transformation matrix, [transform.json](https://storage.cmusatyalab.org/mega-nerf-data/building-pixsfm.json) into the ``TerraSLAM_relay`` folder.
    ```
    cd /root/TerraSLAM_relay
    wget https://storage.cmusatyalab.org/terra-slam/transform.json
    ```
    Similarly, if you want to learn more about how to compute the transformation matrix between SLAM and GPS, please refer to the code and README in the ``SLAM-GPS-align`` subfolder.

### Launch ORB-SLAM3
Inside the TerraSLAM container:
```
ros2 launch orb_slam3_ros2_wrapper unirobot_mono.launch.py
```
This command is used for running the demo with a monocular camera.
Of course, TerraSLAM also supports stereo cameras by using 
```
ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
```

### Launch TerraSLAM Relay
In another terminal, start a new TerraSLAM container processes by 
```
docker exec -it TerraSLAM /bin/bash
```

Then,
```
cd TerraSLAM_relay
python3 relay.py
```

### Publish Images and Run TerraSLAM
In another terminal, start a new TerraSLAM container processes and run
```
cd Database
python3 image_publish.py Mill-video/
``` 
You will see the printed GPS coordinates in the terminal logs of the former ``relay.py``

## 3. TerraSLAM Visualization

### 2D Localization Results Visualization on Google Map
On your local machine (no need to enter a new TerraSLAM container),
```
cd container_root/TerraSLAM_relay
python3 relay_client.py ../Database/Mill-video -s 127.0.0.1 -p 43322
```
This will send the images to the ORB-SLAM3 wrapper for SLAM processing. 
At the same time, a GUI will launch to display the current GPS coordinates and plot the results on a map. 
If you provide a Google API key, the results will be shown on Google Map.

### 3D Localization Results Visualization in Blender
1. Download [Blender version 2.93](https://www.blender.org/download/lts/2-93/). In our tests, Blender versions above 3.1 also work, but we found that Blender 2.93 is a more stable and compatible version across different platforms.
2. Download the [Blender Project File](https://storage.cmusatyalab.org/mega-nerf-data/building-pixsfm.tgz). Make sure that the ``Mill-19.blend`` and the model texture folder ``mill-19-half-q0to84q`` are in the same directory.
This project contains the 3D GIS model of the Mill-19 area, the SLAM point cloud, the alignment between SLAM and GIS models, a drone model, and the Blender scripts interacted with TerraSLAM. 
Import ``Mill-19.blend`` in Blender.
3. In Blender's menu bar, switch to the ``scripting`` workspace. 
In the editor panel, select and run `drone_pose.py`. This will start a Blender server (at ``127.0.0.1:11223``) that receives the drone pose calculated by TerraSLAM from the client and renders the result in the 3D GIS model. After running the script, you can return to the default ``layout`` workspace.
4. Run TerraSLAM as the above and send pose data to blender:
    - In the previous terminals inside the TerraSLAM, run:
      ```
      ros2 launch orb_slam3_ros2_wrapper unirobot_mono.launch.py
      ```
      and
      ```
      cd Database
      python3 image_publish.py Mill-video
      ```
    - In a new terminal inside the TerraSLAM container, run:
      ```
      cd TerraSLAM_runtime
      python3 pose_tcp.py
      ```
    You will see a virtual drone flying in the GIS world!



## [Optional] Running TerraSLAM in Real-Time with Olympe Drones

1. Set up your Olympe development environment [here](https://developer.parrot.com/docs/olympe/installation.html). Using a virtual environment is highly recommanded [here](https://developer.parrot.com/docs/olympe/pip_on_debian_based_distros.html#best-practices).
2. Generate a parameter file of your Olympe parrot. We have prepared one if you want to use:
```bash
cd /root/colcon_ws/orb_slam3_ros2_wrapper/params
cp olympe.yaml.temp olympe.yaml
```
3. Change the ``*.yaml`` parameter file you want to use in ``unirobot_mono.launch.py``
4. Launch ORB-SLAM3: ```ros2 launch orb_slam3_ros2_wrapper unirobot_mono.launch.py```. You should see a window popup which is waiting for images. This is partially indicative of the setup correctly done.
5. Open another terminal and feed real-time drone captured images to ORB-SLAM3 through Ros2. Make sure you are in the virtual Olympe development environment
```bash
cd /root/olympe_dev
python ros2_streaming.py
```

Because the mono type assume the depth based on the computer version so if you find the orbslam3 warning of "not initialized" please shake your camera! 

## [Optional] Running TerraSLAM with a Gazebo Classic simulation

1. Setup the ORB-SLAM3 ROS2 Docker using the steps above. Once you do (1) step in the ```Launching ORB-SLAM3``` section, you should see a window popup which is waiting for images. This is partially indicative of the setup correctly done.
2. Setup the simulation by following the README [here](https://github.com/suchetanrs/scout-husky-gazebo-ros2)
3. Once you are able to teleop the robot, you should be able to run ORB-SLAM3 with both the containers (simulation and wrapper) running parallely.

### Potential issues you may face.
The simulation and the wrapper both have their ```ROS_DOMAIN_ID``` set to 55 so they are meant to work out of the box. However, you may face issues if this environment variable is not set properly. Before you start the wrapper, run ```ros2 topic list``` and make sure the topics namespaced with ```scout_2``` are visible inside the ORB-SLAM3 container provided the simulation is running along the side.

