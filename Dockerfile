# syntax=docker/dockerfile:1
FROM nvidia/cuda:12.4.1-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV CUDACXX=/usr/local/cuda-12.4/bin/nvcc
ENV CUDA_HOME=/usr/local/cuda-12.4
ENV PATH=/usr/local/cuda-12.4/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda-12.4/lib64:${LD_LIBRARY_PATH}

# Base tools, python venv, rosdep and locales
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales curl gnupg2 lsb-release ca-certificates git build-essential \
    python3-pip python3-venv \
    && locale-gen en_US en_US.UTF-8
ENV LANG=en_US.UTF-8


# Add official ROS 2 Humble repository & keyring
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-ackermann-msgs \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*


# Upgrade CMake via pip to satisfy CMakeLists.txt VERSION >= 3.27
RUN python3 -m pip install --no-cache-dir --upgrade pip wheel setuptools cmake

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Setup sim_ws using the dev-humble instructions
WORKDIR /sim_ws
RUN mkdir -p /sim_ws/src \
    && python3 -m venv --system-site-packages /sim_ws/.venv \
    && . /sim_ws/.venv/bin/activate \
    && python3 -m pip install --upgrade pip wheel setuptools

# Clone f1tenth_gym_ros (dev-humble)
WORKDIR /sim_ws/src
RUN git clone -b dev-humble https://github.com/f1tenth/f1tenth_gym_ros.git \
    && cd f1tenth_gym_ros \
    && git clone -b dev-humble https://github.com/f1tenth/f1tenth_gym.git \
    && . /sim_ws/.venv/bin/activate \
    && cd f1tenth_gym && pip install -e .


# Switch shell to bash so ROS 2 bash setup scripts source cleanly
SHELL ["/bin/bash", "-c"]

# Install simulator rosdep dependencies and build
WORKDIR /sim_ws
RUN apt-get update \
    && source /opt/ros/humble/setup.bash \
    && source /sim_ws/.venv/bin/activate \
    && rosdep install -i --from-paths src --rosdistro humble -y \
    && colcon build --symlink-install

# Copy and build rmpc_f1tenth
WORKDIR /rmpc_ws/src/rmpc_f1tenth
COPY . /rmpc_ws/src/rmpc_f1tenth/





# Resolve controller rosdep dependencies and build with CUDA compiler
WORKDIR /rmpc_ws
RUN apt-get update \
    && source /opt/ros/humble/setup.bash \
    && source /sim_ws/.venv/bin/activate \
    && source /sim_ws/install/setup.bash \
    && rosdep install -i --from-paths src --rosdistro humble -y \
    && colcon build --packages-select rmpc_f1tenth \
       --cmake-args -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.4/bin/nvcc

# Entrypoint
COPY docker_entrypoint.sh /docker_entrypoint.sh
RUN chmod +x /docker_entrypoint.sh
ENTRYPOINT ["/docker_entrypoint.sh"]
CMD ["bash"]
