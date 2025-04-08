SlideSlamWs="$WS_PATH" # point to your workspace directory
SlideSlamCodeDir="$WS_PATH/src/SLIDE_SLAM" # point to code dir with SLIDE_SLAM
BAGS_DIR="$WS_PATH/src/SLIDE_SLAM/bags" # point to your bags / data directory

DOCKER_RUN_FLAGS=""

# If not running in CI, add -it
if [ -z "$CI" ]; then
  DOCKER_RUN_FLAGS="-it"
fi

EXTRA_VOLUMES=""
if [ -z "$CI" ]; then
  EXTRA_VOLUMES+=" --gpus=\"all\""
  EXTRA_VOLUMES+=" --env=\"DISPLAY=$DISPLAY\""
  EXTRA_VOLUMES+=" --env=\"QT_X11_NO_MITSHM=1\""
  EXTRA_VOLUMES+=" --env=\"XAUTHORITY=/root/.Xauthority\""
  EXTRA_VOLUMES+=" --volume=\"/tmp/.X11-unix:/tmp/.X11-unix:rw\""
  EXTRA_VOLUMES+=" --volume=\"/home/$USER/.bash_aliases:/root/.bash_aliases\""
  EXTRA_VOLUMES+=" --volume=\"$XAUTHORITY:/root/.Xauthority:ro\""
fi

docker run $DOCKER_RUN_FLAGS \
    --name="slideslam_ros" \
    --net="host" \
    --privileged \
    --workdir="/opt/slideslam_docker_ws" \
    --env="DISPLAY=$DISPLAY" \
    --volume="$SlideSlamWs:/opt/slideslam_docker_ws" \
    --volume="$SlideSlamCodeDir:$SlideSlamCodeDir" \
    --volume="$BAGS_DIR:/opt/bags" \
    --volume="/home/$USER/repos:/home/$USER/repos" \
    $EXTRA_VOLUMES \
    slideslam_ros_image \
    bash

