SlideSlamWs="$WS_PATH" # point to your workspace directory
SlideSlamCodeDir="$WS_PATH/src/SLIDE_SLAM" # point to code dir with SLIDE_SLAM
BAGS_DIR="$WS_PATH/src/SLIDE_SLAM/bags" # point to your bags / data directory

DOCKER_RUN_FLAGS=""

# If not running in CI, add -it
if [ -z "$CI" ]; then
  DOCKER_RUN_FLAGS="-it"
fi

docker run $DOCKER_RUN_FLAGS \
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
    if [ -n "$XAUTHORITY" ]; then
    DOCKER_ARGS="$DOCKER_ARGS -v $XAUTHORITY:/root/.Xauthority:ro"
    fi
    slideslam_ros_image \
    bash

