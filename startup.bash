#!/bin/bash
. /home/docker/ws/install/setup.bash
#command python -m SimpleHTTPServer 8000 &
command roscore -p 11311 &
rosservice call /srcsim/finals/start_task 1 2 
python ./Funstuff/hardCode.py
command python ./dockerTest/Task1.py
sleep 30s
python ./dockerTest/PointCloudTask1.py
