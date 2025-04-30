FROM xurobotics/slide-slam:latest

# Install dependencies
RUN apt-get update && apt-get install -y \
    cmake \
    git \
    libopencv-dev \
    lsof \
    && rm -rf /var/lib/apt/lists/*

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
    $CONDA_DIR/bin/conda  clean -afy

# Activate conda environment temporarily and install ROS dependencies
RUN source $CONDA_DIR/etc/profile.d/conda.sh && \
    conda activate ros310 && \
    pip install rospkg && \
    conda clean -afy

# RSet working directory back to default
WORKDIR /app
