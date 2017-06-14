FROM ros:indigo-robot
#ADD reinstall.shaween reinstall.shaween

#CMD ["./reinstall.shaween"]

#add necssary messages
RUN apt-get update \
 #&& apt-get dist-upgrade \
 && apt-get install -y \
	ros-indigo-desktop=1.1.5-0* \
	ros-indigo-rospy \
	python-numpy \
	python-scipy \
	ros-indigo-cv-bridge \
	ros-indigo-geometry-msgs \
	python-catkin-tools \
	ros-indigo-ihmc-msgs \
	ros-indigo-tf2 \
	build-essential \
	python-catkin-tools \
	ros-indigo-catkin \
	ros-indigo-tf2-ros \
	ros-indigo-tf2-msgs \
	ros-indigo-ros-controllers \
	ros-indigo-ros-control \
	ros-indigo-rosbag \
&& rm -rf /var/lib/apt/lists/*
RUN apt-get dist-upgrade && apt-get update
#RUN catkin_make
#RUN source /opt/ros/indigo/setup.bash
#clone srcsim and set environment variable
RUN sleep 5s
ENV WS /home/docker/ws
RUN mkdir -p ${WS}/src
WORKDIR ${WS}
RUN hg clone https://bitbucket.org/osrf/srcsim ${WS}/src/srcsim
#WORKDIR ~/
ENV PATH $PATH:/opt/ros/indigo/bin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin/bin:.
RUN . /opt/ros/indigo/setup.sh \
	&& catkin config --cmake-args -DBUILD_MSGS_ONLY=True \
	&& catkin config --install \
	&& catkin build
#RUN ["pip", "install", "rospy"]
EXPOSE 8000
EXPOSE 8001
EXPOSE 32000
ENV ROS_MASTER_URI http://127.0.0.1:8001
#mkdir Funstuff
ENV PYTHONPATH = $PYTHONPATH:/opt/ros/indigo/lib/python2.7/dist-packages/:/{WS}/src
ADD ./Funstuff/* ./
#RUN . setup.bash
#RUN catkin clean
#CMD ["./startup.bash"]
#CMD ["echo", "$PATH"]
#RUN rosdep update
RUN ["python", "serverSide.py"]

