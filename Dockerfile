FROM ubuntu:jammy

# ------------[ LOCALES SETUP ]------------
RUN apt-get update && \
    apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

ENV ROS_VERSION=2
ENV ROS_DISTRO=humble
ENV ROS_PYTHON_VERSION=3
ENV TZ=Asia/Jakarta
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# ------------[ ROS2 HUMBLE INSTALLATION ]------------
RUN apt-get update && \
    apt-get install -y software-properties-common curl && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y ros-humble-desktop

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    build-essential \
    git \
    nano \
    iputils-ping \
    wget \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool && \ 
    rm -rf /var/lib/apt/lists/*

RUN rosdep init && \
    rosdep update --rosdistro humble

# ------------[ DYNAMIXEL SDK INSTALLATION ]------------
RUN git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git

WORKDIR /DynamixelSDK/python/

RUN python3 setup.py install

WORKDIR /

RUN rm -rf /DynamixelSDK && \
    apt-get remove -y git

# ------------[ NODEJS INSTALLATION ]------------
RUN apt-get update && \
    apt-get install -y npm lsof && \
    npm install -g node@20.11.1

# ------------[ PYTHON LIBRARIES ]------------
RUN apt-get update && \
    apt-get install -y pip && \
    pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir \ 
    numpy \
    pyyaml \ 
    && \
    apt-get remove -y pip

# ------------[ BASH SETUP ]------------
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "#!/usr/bin/env bash" > /rcl_entrypoint.sh
RUN echo "source /opt/ros/humble/setup.bash" >> /rcl_entrypoint.sh
RUN echo 'exec "$@"' >> /rcl_entrypoint.sh
RUN chmod +x /rcl_entrypoint.sh

# ------------[ ENTRYPOINT AND LOOP ]------------
ENTRYPOINT ["/rcl_entrypoint.sh"]
CMD ["tail", "-f", "/dev/null"]