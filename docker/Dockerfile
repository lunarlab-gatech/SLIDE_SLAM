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

# Install pyproj into the python environment
RUN pip install pyproj=3.5.0

# Set working directory back to default
WORKDIR /app
