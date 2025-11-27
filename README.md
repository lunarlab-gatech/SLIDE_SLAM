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

**Important:** Go to `docker/run_container.sh` in this repository & make sure the following directory is correct:
```
REPO_DIR= ...
```
which should point to the folder of the ros worksapce where this directory is found.

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

**Build the workspace**: 
Navigate to `~/slideslam_original_ws` and then run the following commands:
```
catkin config --cmake-args -Dcatkin_DIR=/opt/ros/noetic/share/catkin/cmake
source /opt/ros/noetic/setup.bash
catkin build -DCMAKE_BUILD_TYPE=Release
```

**Run the demos**
```
source ~/slideslam_original_ws/devel/setup.bash
```
Follow the instructions below to run the demos. Remember to commit your changes inside docker envirnoment to keep them (e.g. newly installed pkgs). 

Type `exit` to exit the container; you can re-enter using `docker/enter_container.sh`.

**Troubleshoot**:
- If you do not see your code inside docker, double check `docker/run_container.sh` file to make sure you have your workspace mapped properly. 


## Run Demos

### Download example data

Please download the processed data bags from [this link](https://drive.google.com/drive/folders/125N7srccxXPmn2HAQFjwXNNzYOO7DoqV). This containes compact processed bags for forest and urban outdoor environments. Please use the right data with the right scripts as specified below.


### What these demos will do
- Intermittent communication between robot nodes at a fixed time interval. 
- Multiple robots running on the same computer, and therefore, the computational load is going to be (num_robots multiplied by the computation load of each robot during actual experiment).

### Run multi-robot demo (based on LiDAR data)
**First, please refer to the section above and make sure you have everything built.**

**Option 1:** Use our tmux script (recommended)

Source and go to the ' folder inside `multi_robot_utils_launch` package:
```
source ~/slideslam_ws/devel/setup.bash
roscd multi_robot_utils_launch/script
```

Modify `tmux_multi_robot_with_bags_forest.sh` to set the `BAG_DIR` to where you downloaded the bags

Modify `BAG_PLAY_RATE` to your desired play rate (lower than 1.0 if you have a low-specification CPU)

Then make it executable if needed
```
chmod +x tmux_multi_robot_with_bags_forest.sh
```

Finally, execute this script
```
./tmux_multi_robot_with_bags_forest.sh
```

If you want to terminate this program, go to the last terminal window and press `Enter` to kill all the tmux sessions.

**Option 2:** If you prefer not to use this tmux script, please refer to the `roslaunch` commands inside this tmux script and execute those commands by yourself.

**To run the same above example with urban outdoor data, use the `tmux_multi_robot_with_bags_parking_lot.sh` script and repeat the above steps.**

## Run on raw sensor data (RGBD or LiDAR bags)
This section will guide you through running our code stack with raw sensor data, which is rosbags containing LiDAR-based or RGBD-based data. Note: size of these raw bags are usually anywhere from 10-100 GB.

### Download example data

Please download the LiDAR demo bags from [this link](https://drive.google.com/drive/folders/1heAnoe6qESp2uQjjwwdR0Q3sUijkcUER). It is present inside the `outdoor` folder.

Please download the RGBD demo bags from [this link](https://drive.google.com/drive/folders/1heAnoe6qESp2uQjjwwdR0Q3sUijkcUER). It is present inside the `indoor` folder.

Please download the KITTI benchmark processed bags from [this link](https://drive.google.com/drive/folders/1heAnoe6qESp2uQjjwwdR0Q3sUijkcUER). It is present inside the `kitti_bags` folder.

Please download our trained RangeNet++ model from [this link](https://drive.google.com/drive/folders/1ignTNFZe3KLh9Fy6fakPwwJLEEnV-0E0). It is currently named `penn_smallest.zip`. Follow the instructions in the `Run our LiDAR data experiments`  section below on how to use this model.

### Run our RGBD data experiments

**Option 1:** Use our tmux script (recommended)

Source and go to the ' folder inside `multi_robot_utils_launch` package:
```
source ~/slideslam_original_ws/devel/setup.bash
roscd multi_robot_utils_launch/script
```

Modify `tmux_single_indoor_robot.sh` to set the `BAG_DIR` to where you downloaded the bags. Then run the following:
```
tmuxp load src/SLIDE_SLAM/backend/multi_robot_utils_launch/tmux/tmux_single_indoor_robot.yaml
```


**IMPORTANT**: If it is your first time to run this script, the front-end instance segmentation network will download the weights from the internet. This may take a while depending on your internet speed. Once this is finished, kill all the tmux sessions (see below) and re-run the script.

If you want to terminate this program, go to the last terminal window and press `Enter` to kill all the tmux sessions.

**Option 2:** If you prefer not to use this tmux script, please refer to the `roslaunch` commands inside this tmux script and execute those commands by yourself, or using the detailed instructions found [here](https://github.com/XuRobotics/SLIDE_SLAM/wiki#run-rgbd-raw-bags-detailed-instructions).

### Run our LiDAR Data experiments

**Download the LiDAR semantic segmentation RangeNet++ model**
```
(1) Download the model from the above link.
(2) Unzip the file and place the model in a location of your choice.
(3) Open the extracted model folder and make sure that there are no files inside having a .zip extension. If there are, then rename ALL OF THEM to remove the .zip extension. For example backbone.zip should be renamed to backbone
```

**Option 1:** Use our tmux script (recommended)

Make sure you edit the ```infer_node_params.yaml``` file present inside the ```scan2shape_launch/config``` folder and set the value of ```model_dir``` param to point to the path to the RangeNet++ model you downloaded in the previous step. Make sure to compelte the path with the ```/``` at the end.

Source and go to the ' folder inside `multi_robot_utils_launch` package:
```
source ~/slideslam_ws/devel/setup.bash
roscd multi_robot_utils_launch/script
```

Modify `tmux_single_outdoor_robot.sh` to set the `BAG_DIR` to where you downloaded the bags

Modify `BAG_PLAY_RATE` to your desired play rate (lower than 1.0 if you have a low-specification CPU)

Then make it executable if needed
```
chmod +x tmux_single_outdoor_robot.sh
```

Finally, execute this script
```
./tmux_single_outdoor_robot.sh
```

If you want to terminate this program, go to the last terminal window and press `Enter` to kill all the tmux sessions.

**Option 2:** If you prefer not to use this tmux script, please refer to the `roslaunch` commands inside this tmux script and execute those commands by yourself, or using the detailed instructions found [here](https://github.com/XuRobotics/SLIDE_SLAM/wiki#run-lidar-raw-bags-detailed-instructions).

### Run KITTI Benchmark experiments

**Option 1:** Use our tmux script

Source and go to the ' folder inside `multi_robot_utils_launch` package:
```
source ~/slideslam_ws/devel/setup.bash
roscd multi_robot_utils_launch/script
```

Modify `tmux_single_outdoor_kitti.sh` to set the `BAG_DIR` to where you downloaded the bags

Then make it executable if needed
```
chmod +x tmux_single_outdoor_kitti.sh
```

Finally, execute this script
```
./tmux_single_outdoor_kitti.sh
```

If you want to terminate this program, go to the last terminal window and press `Enter` to kill all the tmux sessions.

## Troubleshoot
**Rate of segmentation:**
- When running on your own data, we recommend to throttle the segmentation topic (segmented point cloud or images) rate to 2-4 Hz to avoid computation delay in the front end, especially if you’re experiencing performance issues at higher rates. Please also update the `expected_segmentation_frequency` parameter in the corresponding `process_cloud_node_*_params.yaml` file as well as the `desired_frequency` in the `infer_node_params.yaml` to the actual rate of the topic. 

# Acknowledgement
We use GTSAM as the backend. We thank [Guilherme Nardari](linkedin.com/in/guilherme-nardari-23ba91a8) for his contributions to this repository. 

# Citation
If you find our system or any of its modules useful for your academic work, we would appreciate it if you could cite our work as follows:
```
@article{liu2024slideslam,
  title={Slideslam: Sparse, lightweight, decentralized metric-semantic slam for multi-robot navigation},
  author={Liu, Xu and Lei, Jiuzhou and Prabhu, Ankit and Tao, Yuezhan and Spasojevic, Igor and Chaudhari, Pratik and Atanasov, Nikolay and Kumar, Vijay},
  journal={arXiv preprint arXiv:2406.17249},
  year={2024}
}
```
