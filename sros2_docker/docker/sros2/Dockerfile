FROM ubuntu:xenial

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu xenial main" > /etc/apt/sources.list.d/gazebo-latest.list' && \
    apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# upgrade distro
RUN apt-get -qq update && \
    apt-get -qq dist-upgrade -y

# install deps
RUN apt-get -qq update && \
    apt-get -qq install -y \
      git \
      wget \
      build-essential \
      cppcheck \
      cmake \
      libopencv-dev \
      libpoco-dev \
      libpocofoundation9v5 \
      libpocofoundation9v5-dbg \
      python-empy \ 
      python3-dev \
      python3-empy \
      python3-nose \
      python3-pip \
      python3-setuptools \
      python3-vcstool \
      clang-format \
      pydocstyle \
      pyflakes \
      python3-coverage \
      python3-mock \
      python3-pep8 \
      uncrustify \
      openssl \
      libssl-dev
      

RUN pip3 install flake8 \
      flake8-import-order

RUN echo "hmmm maybe that would work"
# setup ros2 workspace
ENV ROS2_WS=/root/sros2_ws
RUN rm -rf $ROS2_WS
RUN mkdir -p $ROS2_WS/src
WORKDIR $ROS2_WS

RUN wget https://raw.githubusercontent.com/ros2/sros2/sros2_demo/sros2.repos
RUN echo "hooray lets move forward"

RUN vcs import src < sros2.repos

WORKDIR $ROS2_WS/src/eProsima
RUN rm -rf Fast-RTPS && git clone --recursive https://github.com/eProsima/Fast-RTPS.git -b release/1.4.0

WORKDIR $ROS2_WS
RUN src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install --isolated --cmake-args -DSECURITY=ON -- --parallel

# RUN /bin/bash -c "source $ROS2_WS/install_isolated/local_setup.bash" && \
#     sros2 create_keystore demo_keys && \
#     sros2 create_key demo_keys talker && \
#     sros2 create_key demo_keys listener

RUN echo "export ROS2_WS=/root/sros2_ws" >> /root/.bashrc && \
    echo "export ROS_SECURE_ROOT=/root/sros2_ws/demo_keys" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /root/.bashrc && \
    echo "source $ROS2_WS/install_isolated/local_setup.bash" >> /root/.bashrc
