FROM ubuntu:xenial

RUN apt-get -qq update && \
    apt-get -qq install locales -y

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG en_US.UTF-8

RUN sh -c 'echo "deb http://repo.ros2.org/ubuntu/main xenial main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# upgrade distro
RUN apt-get -qq update && \
    apt-get -qq dist-upgrade -y

# install deps
RUN apt-get -qq update && \
    apt-get -qq install -y \
      python3-pip \
      ros-r2b2-demo-nodes-cpp \
      ros-r2b2-demo-nodes-py \
      ros-r2b2-ros2run \
      ros-r2b2-sros2 \
      vim

RUN pip3 install argcomplete

# setup ros2 workspace
ENV ROS2_WS=/root/sros2_demo
WORKDIR $ROS2_WS

RUN echo "export ROS_SECURITY_ROOT_DIRECTORY=/root/sros2_demo/demo_keys" >> /root/.bashrc && \
    echo "export ROS_SECURITY_ENABLE=true" >> /root/.bashrc && \
    echo "export ROS_SECURITY_STRATEGY=Enforce" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /root/.bashrc && \
    echo "source /opt/ros/r2b2/setup.bash" >> /root/.bashrc
