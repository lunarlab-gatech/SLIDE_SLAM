SlideSlamWs="/home/dbutterfield3/slideslam_docker_ws" # point to your workspace directory
SlideSlamCodeDir="/home/dbutterfield3/slideslam_docker_ws/src/SLIDE_SLAM" # point to your code directory where you cloned the repository
BAGS_DIR='/home/dbutterfield3/slideslam_docker_ws/src/SLIDE_SLAM' # point to your bags / data directory

xhost +local:root # for the lazy and reckless
docker run -it \
    --name="slideslam_ros" \
    --net="host" \
    --privileged \
    --gpus="all" \
    --workdir="/opt/slideslam_docker_ws" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$SlideSlamWs:/opt/slideslam_docker_ws" \
    --volume="$SlideSlamCodeDir:$SlideSlamCodeDir" \
    --volume="$BAGS_DIR:/opt/bags" \
    --volume="/home/$USER/.bash_aliases:/root/.bash_aliases" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/$USER/repos:/home/$USER/repos" \
    xurobotics/slide-slam:latest \
    bash

