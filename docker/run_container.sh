REPO_DIR="/mnt/c/Users/camer/Documents/LunarLab/slide_slam_mk2"
DATA_DIR="/mnt/d/Hercules/V1.4.1/data/bags"

export XAUTHORITY=$HOME/.Xauthority

docker run -it \
    --name="slideslam_scorched_earth" \
    --net="host" \
    --privileged \
    --gpus="all" \
    --workdir="/home/$USER/slideslam_original_ws" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=/tmp/.Xauthority" \
    --env="USER_ID=$(id -u)" \
    --env="GROUP_ID=$(id -g)" \
    --volume="$REPO_DIR:/home/$USER/slideslam_original_ws" \
    --volume="$DATA_DIR:/home/$USER/data" \
    --volume="/home/$USER/.bash_aliases:/home/$USER/.bash_aliases" \
    --volume="/home/$USER/.ssh:/home/$USER/.ssh:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/$USER/repos:/home/$USER/repos" \
    --volume="$XAUTHORITY:/tmp/.Xauthority:ro" \
    slideslam_scorched_earth \
    bash