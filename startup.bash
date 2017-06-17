#!/bin/bash
. /home/docker/ws/install/setup.bash
#command python -m SimpleHTTPServer 8000 &
command roscore -p 11311 &
python ./Funstuff/serverSide.py && ./Funstuff/sender.py && ./dockerTest/serverSide.py
