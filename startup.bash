#!/bin/bash
source /home/docker/ws/install/setup.bash
python -m SimpleHTTPServer 8000 &
roscore -p 8001
#python ./serverSide.py
