#!/bin/bash

# NAOQI
#YOLO ROS for Pepper
#--------------------------------


mkdir -p ~/YOLO_ws/src
git clone --recursive https://github.com/ReyesDeJong/darknet_ros.git
cd darknet_ros
git checkout pepper
cd ~/YOLO_ws/
catkin_make -DCMAKE_BUILD_TYPE=Release
cd ..
tar czf YOLO_ws.tar.gz YOLO_ws

# From pc terminal launch something like 'scp -P 2222 nao@127.0.0.1:/home/nao/YOLO_ws.tar.gz /home/asceta/' to get YOLO_ws for Pepper

