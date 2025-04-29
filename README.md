# SlideSLAM + ROMAN + Relative Inter Robot Factors


# Installation with Docker 

**Install docker**: https://docs.docker.com/desktop/install/linux/ubuntu/#install-docker-desktop

**Pull the docker image**: 
```
docker pull xurobotics/slide-slam:latest
```

**Create the workspace (important)**
```
mkdir -p ~/slideslam_docker_ws/src
cd ~/slideslam_docker_ws/src
```
_Creating the workspace outside the docker helps you keep your files and changes within the workspace even if you delete the un-committed docker container._

**Clone the repo**: 
```
git clone https://github.com/XuRobotics/SLIDE_SLAM.git
cd ~/slideslam_docker_ws/src/SLIDE_SLAM
chmod +x run_slide_slam_docker.sh
```

**Install various dependencies**:
```
cd ~/slideslam_docker_ws/src
git clone git@github.com:lunarlab-gatech/roman_ros.git
git clone git@github.com:eric-wieser/ros_numpy.git
cd ~/slideslam_docker_ws/scripts
git@github.com:lunarlab-gatech/roman.git
```

**(Optional) Only if you need to run on LiDAR data, install Faster-LIO and LiDAR drivers**: 
```
cd ~/slideslam_docker_ws/src
git clone git@github.com:ouster-lidar/ouster_example.git && cd ouster_example && git checkout 43107a1 && cd ..
git clone git@github.com:XuRobotics/faster-lio
git clone git@github.com:KumarRobotics/ouster_decoder.git && cd ouster_decoder && git checkout d66b52d  && cd ..

```
*Find the ```CMakeLists.txt``` in ```ouster_decoder``` and comment out the last three lines (the ```ouster_viz```) to avoid fmt issue*

**Build the docker image**:

Run the ```build_image.sh``` file.

**Run the docker image**: 

**Important:** Go to `./run_container_dbutter.sh`, make sure the following three directories are correct
```
SlideSlamWs="/home/sam/slideslam_docker_ws"
```
should point to your workspace directory
```
SlideSlamCodeDir="/home/sam/slideslam_docker_ws/src/SLIDE_SLAM"
``` 
should point to your code directory where you cloned the repository
```
BAGS_DIR="/home/sam/bags"
```
should point to your bags (data) directory

Then run:
```
./run_container_dbutter.sh
```

**Install ROMAN into the Python 3.10 environment**:
```
source /opt/conda/etc/profile.d/conda.sh
conda activate ros310
cd /opt/slideslam_docker_ws/scripts/roman
./install.sh
```

**Build the workspace**: 
Now, in a new terminal (VERY IMPORTANT):
```
cd /opt/slideslam_docker_ws
catkin build -DCMAKE_BUILD_TYPE=Release
```

Note: If you are having errors, try running the following command first:
```
catkin clean -y
```

**Run the demos**
```
source /opt/slideslam_docker_ws/devel/setup.bash
```
Follow the instructions below to run the demos. Remember to commit your changes inside docker envirnoment to keep them (e.g. newly installed pkgs). 

Type `exit` to exit the container.

You can re-enter the container, or enter the container from a new terminal by either 
```
docker start slideslam_ros && docker exec -it slideslam_ros /bin/bash
``` 
or remove your docker container using the command 
```
docker rm slideslam_ros
``` 
before you run the docker image again.


**Troubleshoot**:
- If you do not see your code or bags inside docker, double check `run_slide_slam_docker.sh` file to make sure you have your workspace and BAG folders mapped properly. 


# Run Test Cases

Use the following commands to run the test cases for this repository:

```
catkin build -DCMAKE_BUILD_TYPE=Debug --catkin-make-args tests
catkin run_tests
```

Make sure to rebuild with `-DCMAKE_BUILD_TYPE=Release` when you want to run the algorithm as normal.

