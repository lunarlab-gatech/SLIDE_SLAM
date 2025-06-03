SlideSlamWs="$WS_PATH" # point to your workspace directory
SlideSlamCodeDir="$WS_PATH/src/SLIDE_SLAM" # point to code dir with SLIDE_SLAM
BAGS_DIR="$WS_PATH/src/SLIDE_SLAM/bags" # point to your bags / data directory

docker run -d \
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
    bash -c "while true; do sleep 60; done"

