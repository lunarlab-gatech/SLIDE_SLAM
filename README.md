# SlideSLAM

This repository contains the source code for the project SlideSLAM: Sparse, Lightweight, Decentralized Metric-Semantic SLAM for Multi-Robot Navigation; altered for use as a baseline for the Lunar Lab.

## Installation with Docker

**Install docker**: https://docs.docker.com/desktop/install/linux/ubuntu/#install-docker-desktop

**Pull the docker image**: 
```
docker pull xurobotics/slide-slam:latest
```

**Create the workspace (important)**

First, create the ros workspace with the `src` folder, and then navigate the terminal to that `src` folder.

_Creating the workspace outside the docker helps you keep your files and changes within the workspace even if you delete the un-committed docker container._

**Clone the repo**: 
```
git clone https://github.com/XuRobotics/SLIDE_SLAM.git
```

**(Optional) Only if you need to run on LiDAR data, install Faster-LIO and LiDAR drivers**: 

Navigate to the root folder of the ros workspace and then run:
```
git clone git@github.com:ouster-lidar/ouster_example.git && cd ouster_example && git checkout 43107a1 && cd ..
git clone git@github.com:XuRobotics/faster-lio
git clone git@github.com:KumarRobotics/ouster_decoder.git && cd ouster_decoder && git checkout d66b52d  && cd ..
```
*Find the ```CMakeLists.txt``` in ```ouster_decoder``` and comment out the last three lines (the ```ouster_viz```) to avoid fmt issue*

**Run the docker image**: 

**Important:** Go to `docker/run_container.sh` in this repository & make sure the following directories is correct:
```
REPO_DIR= ...
DATA_DIR= ...
```
These should point to the folder of the ros workspace where this directory is found, and the directory where datasets are located.

Additionally, update the `docker/Dockerfile` with the corresponding values for your user:
```
# Set user arguments (being username, echo $UID, and id -g)
ARG USERNAME=
ARG USER_UID=
ARG USER_GID=
```

Then run the following in the root folder of this repository:
```
./docker/build_image.sh
./docker/run_container.sh
```

The rest of this README **assumes that you are inside the Docker container**. For easier debugging and use, its highly recommended to install the [VSCode Docker extension](https://code.visualstudio.com/docs/containers/overview), which allows you to start/stop the container and additionally attach VSCode to the container by right-clicking on the container and selecting `Attach Visual Studio Code`. If that isn't possible, you can re-enter the container running the following command:
```
./docker/enter_container.sh
```

**Build the workspace**: 
Navigate to `~/slideslam_original_ws` and then run the following commands:
```
catkin config --cmake-args -Dcatkin_DIR=/opt/ros/noetic/share/catkin/cmake
source /opt/ros/noetic/setup.bash
catkin build -DCMAKE_BUILD_TYPE=Release
```

Before running any code, make sure to source the following file:
```
source ~/slideslam_original_ws/devel/setup.bash
```

**Troubleshoot**:
- If you do not see your code inside docker, double check `docker/run_container.sh` file to make sure you have your workspace mapped properly. 

## Run HERCULES experiments

Run the following commands:
```
source ~/slideslam_original_ws/devel/setup.bash
tmuxp load src/SLIDE_SLAM/backend/multi_robot_utils_launch/tmux/hercules.yaml
```