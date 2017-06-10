FROM ros:indigo-ros-base


#add necssary messages
RUN apt-get update \
 && apt-get install -y \
	build-essential \
	python-catkin-tools \
	ros-indigo-catkin \
	ros-indigo-ihmc-msgs \
	ros-indigo-rosbag \
	ros-indigo-tf \
	ros-indigo-tf2 \
	ros-indigo-tf2-geometry-msgs \

&& rm -rf /var/lib/apt/lists/*


#clone srcsim and set environment bariable
ENV WS /home/docker/ws
RUN mkdir -p{WS}/src
WORKDIR ${WS}
RUN hg clone https://bitbucket.org/osrf/srcsim ${WS}/src/srcsim


EXPOSE 8000
EXPOSE 8001
EXPOSE 32000
ENV ROS_MASTER_URI http://127.0.0.1:8001

ADD startup.bash startup.bash
CMD ["python", "./Funstuff/serverSide.py"]
CMD ["./startup.bash"]


