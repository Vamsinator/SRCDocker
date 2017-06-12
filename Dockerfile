FROM ros:indigo-ros-base
#ADD reinstall.shaween reinstall.shaween

#CMD ["./reinstall.shaween"]

#add necssary messages
RUN apt-get update \
 && apt-get install -y \
	ros-indigo-desktop=1.1.5-0* \
	ros-indigo-rospy \
	python-numpy \
	python-scipy \
&& rm -rf /var/lib/apt/lists/*
#RUN catkin_make
#RUN source /opt/ros/indigo/setup.bash
#clone srcsim and set environment variable
RUN sleep 5s
ENV WS /home/docker/ws
RUN mkdir -p ${WS}/src
WORKDIR ${WS}
RUN hg clone https://bitbucket.org/osrf/srcsim ${WS}/src/srcsim
WORKDIR ~/
#RUN ["pip", "install", "rospy"]
EXPOSE 8000
EXPOSE 8001
EXPOSE 32000
ENV ROS_MASTER_URI http://127.0.0.1:8001]
#mkdir Funstuff
ADD ./Funstuff/ ./
ADD ./dockerTest ./
ADD startup.bash startup.bash
ENV PYTHONPATH = $PYTHONPATH:/opt/ros/indigo/lib/python2.7/dist-packages/
#RUN ["python", "./Funstuff/serverSide.py"]
#CMD ["./startup.bash"]
#CMD ["echo", "$PATH"]
RUN ["python", "serverSide.py"]

