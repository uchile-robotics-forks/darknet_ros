# YOLO v2 ROS: Real-Time Object Detection with ROS for Pepper

## Overview

This is a ROS package developed for object detection in camera images, modified specifically to work with Pepper robot. This project is based on [YOLO v2 for ROS: Real-Time Object Detection for ROS] of [leggedrobotics](https://github.com/leggedrobotics/darknet_ros), where raw installations for any robot with ROS can be found.

In next paragraphs the original project readme is slightly modified to summarice the work done and how to manage the package.



## Citing

The YOLO methods used in this software are described in the paper: [You Only Look Once: Unified, Real-Time Object Detection](https://arxiv.org/abs/1506.02640).

## Installation

### Dependencies

This software is built on the Robotic Operating System ([ROS]), which needs to be [installed](http://wiki.ros.org) first, it also considers that you have the latest Pepper virtual machine (VM) with OpenCV installed. Additionally, YOLO for ROS depends on following software:

- [OpenCV](http://opencv.org/) (computer vision library),
- [boost](http://www.boost.org/) (c++ library),

### Building

[![Build Status](https://ci.leggedrobotics.com/buildStatus/icon?job=github_leggedrobotics/darknet_ros/master)](https://ci.leggedrobotics.com/job/github_leggedrobotics/job/darknet_ros/job/master/)

In order to install darknet_ros, clone the latest version from this repository into your catkin workspace, switch to the pepper branch and compile the package using ROS inside the Pepper VM.

    cd catkin_workspace/src
    git clone --recursive https://github.com/ReyesDeJong/darknet_ros.git
    git checkout pepper
    cd ../

To maximize performance, make sure to build in *Release* mode. You can specify the build type by setting

    catkin_make -DCMAKE_BUILD_TYPE=Release

### Download weights

The tiny-yolo-voc.weights are downloaded automatically in the CMakeLists.txt file. If you need to download weights, go into the weights folder and download the two pre-trained weights from the VOC data set:

    cd catkin_workspace/src/darknet_ros/darknet_ros/yolo_network_config/weights/
    wget http://pjreddie.com/media/files/yolo-voc.weights
    wget http://pjreddie.com/media/files/tiny-yolo-voc.weights


To use the COCO detection objects, you can download the following weights:

    cd catkin_workspace/src/darknet_ros/darknet_ros/yolo_network_config/weights/
    wget http://pjreddie.com/media/files/yolo.weights
    wget http://pjreddie.com/media/files/tiny-yolo.weights

To use the YOLO9000 detection objects, you can download the following weights:

    cd catkin_workspace/src/darknet_ros/darknet_ros/yolo_network_config/weights/
    wget http://pjreddie.com/media/files/yolo9000.weights

### Use your own detection objects

In order to use your own detection objects you need to provide your weights and your cfg file inside the directories:

    catkin_workspace/src/darknet_ros/darknet_ros/yolo_network_config/weights/
    catkin_workspace/src/darknet_ros/darknet_ros/yolo_network_config/cfg/

In addition, you need to create your config file for ROS where you define the names of the detection objects. You need to include it inside:

    catkin_workspace/src/darknet_ros/darknet_ros/config/

Then in the launch file you have to point to your new config file in the line:

    <rosparam command="load" ns="darknet_ros" file="$(find darknet_ros)/config/your_config_file.yaml"/>

By default tiny-yolo-voc.yalm is set, and there is cfg and yalm to an arquitecture that it's lighter than yolo, and can detectect only persons if trained.


## Basic Usage

In order to get YOLO ROS: Real-Time Object Detection for ROS to run with your robot, you will need to adapt a few parameters. It is the easiest if duplicate and adapt all the parameter files that you need to change from the `darkned_ros` package. These are specifically the parameter files in `config` and the launch file from the `launch` folder.

## Nodes

### Node: darknet_ros

This is the main YOLO ROS: Real-Time Object Detection for ROS node. It uses the camera measurements to detect pre-learned objects in the frames.

### ROS related parameters

You can change the names and other parameters of the publishers, subscribers and actions inside `darkned_ros/config/ros.yaml`.

#### Subscribed Topics

* **`/camera_reading`** ([sensor_msgs/Image])

    The camera measurements.
    This topic can hear to the camera of Pepper, by setting: 'topic: /maqui/camera/front/image_raw'
    or a webcam from a computer by setting: 'topic: /usb_cam/image_raw'

#### Published Topics

* **`object_detector`** ([std_msgs::Int8])

    Publishes the number of detected objects.

* **`bounding_boxes`** ([darknet_ros_msgs::BoundingBoxes])

    Publishes an array of bounding boxes that gives information of the position and size of the bounding box in pixel coordinates.

* **`detection_image`** ([sensor_msgs::Image])

    Publishes an image of the detection image including the bounding boxes.

#### Actions

* **`camera_reading`** ([sensor_msgs::Image])

    Sends an action with an image and the result is an array of bounding boxes.

### Detection related parameters

You can change the parameters that are related to the detection by adding a new config file that looks similar to `darkned_ros/config/yolo.yaml`.

* **`image_view/enable_opencv`** (bool)

    Enable or disable the open cv view of the detection image including the bounding boxes.

* **`image_view/use_darknet`** (bool)

    Use the open cv image view from the original darknet algorithm by setting to true or use the on that is implemented in darknet_ros by setting to false.

* **`image_view/wait_key_delay`** (int)

    Wait key delay in ms of the open cv window.

* **`yolo_model/config_file/name`** (string)

    Name of the cfg file of the network that is used for detection. The code searches for this name inside `darkned_ros/yolo_network_config/cfg/`.

* **`yolo_model/weight_file/name`** (string)

    Name of the weights file of the network that is used for detection. The code searches for this name inside `darkned_ros/yolo_network_config/weights/`.

* **`yolo_model/threshold/value`** (float)

    Threshold of the detection algorithm. It is defined between 0 and 1.

* **`yolo_model/detection_classes/names`** (array of strings)

    Detection names of the network used by the cfg and weights file inside `darkned_ros/yolo_network_config/`.
