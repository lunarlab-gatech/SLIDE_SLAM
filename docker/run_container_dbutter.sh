SlideSlamWs="/home/dbutterfield3/slideslam_docker_ws" # point to your workspace directory
SlideSlamCodeDir="/home/dbutterfield3/slideslam_docker_ws/src/SLIDE_SLAM" # point to code dir with SLIDE_SLAM
BAGS_DIR='/home/dbutterfield3/slideslam_docker_ws/src/SLIDE_SLAM/bags' # point to your bags / data directory

docker run -it \
    --name="slideslam_ros" \
    --net="host" \
    --privileged \
    --gpus="all" \
    --workdir="/opt/slideslam_docker_ws" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=/root/.Xauthority" \
    --volume="$SlideSlamWs:/opt/slideslam_docker_ws" \
    --volume="$SlideSlamCodeDir:$SlideSlamCodeDir" \
    --volume="$BAGS_DIR:/opt/bags" \
    --volume="/home/$USER/.bash_aliases:/root/.bash_aliases" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/$USER/repos:/home/$USER/repos" \
    --volume="$XAUTHORITY:/root/.Xauthority:ro" \
    slideslam_ros_image \
    bash

