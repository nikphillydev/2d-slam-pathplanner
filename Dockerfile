FROM osrf/ros:noetic-desktop-full
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    git \
    tmux \
    ros-noetic-jackal-desktop \
    ros-noetic-jackal-simulator \
    ros-noetic-jackal-navigation \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN rosdep update

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "alias die='tmux kill-server'" >> ~/.bashrc

WORKDIR /root/catkin_ws

CMD ["/bin/bash"]