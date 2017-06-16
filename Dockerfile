FROM ros:indigo-ros-base
USER root
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
#RUN apt-get install srcsim
# build srcsim messages
RUN . /opt/ros/indigo/setup.sh \
 && catkin config --cmake-args -DBUILD_MSGS_ONLY=True \
 && catkin config --install \
 && catkin build 
# include bag file with footsteps preprogrammed
# ENV _CATKIN_SETUP_DIR = $_CATKIN_SETUP_DIR:${WS}/install
#RUN   . /home/docker/ws/install/setup.sh
#EXPOSE 8000
#EXPOSE 8001
EXPOSE 32000
EXPOSE 11311
ENV ROS_MASTER_URI http://127.0.0.1:11311
ENV PYTHONPATH = $PYTHONPATH:opt/ros/indigo/lib/python2.7/dist-packages:${WS}/src/srcsim
ENV PATH = $PATH:/opt/ros/indigo/bin
#RUN . /home/docker/ws/install/setup.bash
# startup script
# simple HTTP server and a roscore
ADD Funstuff/* ${WS}/
ADD dockerTest/ ${WS}/
ADD startup.bash startup.bash
RUN chmod +x startup.bash
#CMD . /home/docker/ws/install/setup.sh
ENTRYPOINT ["./startup.bash"]
#CMD python serverSide.py
#CMD python serverSide.py && sender.py 

