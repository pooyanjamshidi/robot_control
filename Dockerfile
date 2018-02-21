FROM cmu-mars/cp1_base

ENV PYTHON_VERSION=3.6.4

RUN sudo apt-get update
RUN sudo apt-get install -y ros-kinetic-gazebo-ros-pkgs \
                            ros-kinetic-gazebo-ros-control \
                            ros-kinetic-kobuki-gazebo \
                            apt-utils \
                            ros-kinetic-gazebo-msgs

ENV ROS_NAVIGATION_MSGS_VERSION 1.13.0
RUN wget -q "https://github.com/ros-planning/navigation_msgs/archive/${ROS_NAVIGATION_MSGS_VERSION}.tar.gz" && \
    tar -xvf "${ROS_NAVIGATION_MSGS_VERSION}.tar.gz" && \
    rm "${ROS_NAVIGATION_MSGS_VERSION}.tar.gz" && \
    mv "navigation_msgs-${ROS_NAVIGATION_MSGS_VERSION}" navigation_msgs && \
    rm navigation_msgs/README.md && \
    mv navigation_msgs/* src && \
    rm -rf navigation_msgs

RUN sudo apt-get install -y python3
RUN sudo apt-get install -y python3-pip

RUN sudo pip3 install --no-cache-dir -r requirements.txt