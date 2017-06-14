FROM ros:indigo-ros-base

# add ihmc messages
RUN apt-get update \
 && apt-get install -y \
    build-essential \
    python-catkin-tools \
    ros-indigo-catkin \
    ros-indigo-ihmc-msgs \
    ros-indigo-rosbag \
    ros-indigo-tf \
    ros-indigo-tf2 \
	ros-indigo-rospy \
	python-numpy \
	python-scipy \
	ros-indigo-cv-bridge \
	ros-indigo-geometry-msgs \
	ros-indigo-ros-controllers \
	ros-indigo-ros-control \
 && rm -rf /var/lib/apt/lists/*

# clone srcsim
ENV WS /home/docker/ws
RUN mkdir -p ${WS}/src
WORKDIR ${WS}
RUN hg clone https://bitbucket.org/osrf/srcsim ${WS}/src/srcsim

# build srcsim messages
RUN . /opt/ros/indigo/setup.sh \
 && catkin config --cmake-args -DBUILD_MSGS_ONLY=True \
 && catkin config --install \
 && catkin build

# include bag file with footsteps preprogrammed

EXPOSE 8000
EXPOSE 8001
ENV ROS_MASTER_URI http://127.0.0.1:8001
ENV PYTHONPATH = $PYTHONPATH:opt/ros/indigo/lib/python2.7/dist-packages
# startup script
# simple HTTP server and a roscore
ADD startup.bash startup.bash
ADD Funstuff/* ./
ADD dockerTest/ ./
CMD python Task1.py

