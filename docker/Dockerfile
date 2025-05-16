FROM xurobotics/slide-slam:latest

# Install dependencies
RUN apt-get update && apt-get install -y \
    cmake \
    git \
    libopencv-dev \
    lsof \
    locales \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Foxy (but don't source it)
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8
RUN sudo add-apt-repository universe && \
    sudo apt update && sudo apt install curl -y && \
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    sudo apt update && \
    sudo apt upgrade -y && \
    sudo apt install ros-foxy-desktop python3-argcomplete ros-dev-tools -y

# Clone and build AprilTag
WORKDIR /opt
RUN git clone https://github.com/AprilRobotics/apriltag.git && \
    cd apriltag && \
    cmake -B build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build --target install && \
    ldconfig

# Install dependencies into python
RUN pip install pyproj==3.5.0
RUN pip install tmuxp

# Install miniconda (take care NOT to add to PATH or will break build)
ENV CONDA_DIR=/opt/conda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O miniconda.sh && \
    bash miniconda.sh -b -p $CONDA_DIR && \
    rm miniconda.sh && \
    $CONDA_DIR/bin/conda clean -afy
RUN $CONDA_DIR/bin/conda  create -n ros310 python=3.10 -y && \
    $CONDA_DIR/bin/conda  clean -afy
RUN $CONDA_DIR/bin/conda config --set auto_activate_base false

# Create a new Conda environment with Python 3.10 for ROMAN
RUN $CONDA_DIR/bin/conda  create -n ros310 python=3.10 -y && \
    $CONDA_DIR/bin/conda  clean -afy && \
    source ${CONDA_DIR}/etc/profile.d/conda.sh && \
    conda activate ros310 && \
    pip install -y numpy scipy open3d rospkg robotdatapy

# TODO: Install ROMAN and Dependencies into new Conda environment

# RSet working directory back to default
WORKDIR /app
