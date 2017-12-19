# YOLO v2 ROS: Real-Time Object Detection with ROS for Jetson TK1 card (and Pepper connection)

## Overview

This is a ROS package developed for object detection in camera images, modified specifically to work with a Jetson TK1. This project is based on [YOLO v2 for ROS: Real-Time Object Detection for ROS] of [leggedrobotics](https://github.com/leggedrobotics/darknet_ros), where raw installations for YOLO with ROS can be found.

In next paragraphs the original project readme is slightly modified to summarize the work done and how to manage the package.

The YOLO packages have been tested under ROS indigo and Ubuntu 14.04 in a Jetson TK1.

## Citing

The YOLO methods used in this software are described in the paper: [You Only Look Once: Unified, Real-Time Object Detection](https://arxiv.org/abs/1506.02640).

## Installation

### Dependencies

This software is built on the Robotic Operating System ([ROS]), which needs to be [installed](http://wiki.ros.org), it also considers that you have installed OpenCV. Additionally, YOLO for ROS depends on following software:

- [OpenCV](http://opencv.org/) (computer vision library),
- [boost](http://www.boost.org/) (c++ library),

To install ROS address to [ROSwiki](http://wiki.ros.org/NvidiaJetsonTK1) for a ROS indigo installation on the Jetson TK1. In the ROSwiki steps por a general setup are listed, where you have to install latest JetPack, Grinch Kernel and Opencv4Tegra, the last 2 are absolutely neccessary for this project. For Grinch installation it's recommended to follow this simple [guide](http://www.jetsonhacks.com/2015/05/26/install-grinch-kernel-for-l4t-21-3-on-nvidia-jetson-tk1/).

Once done the previous steps, you are ready to install ROS indigo, desktop version, following [ROS install](http://wiki.ros.org/indigo/Installation/UbuntuARM) instructions.

### Previous Configurations

After installing ROS indigo, some dependencies issues must be fixed. First extensions of libopencv packages in ROS indigo have to be modified, if you throuw ` grep -R libopencv_` on your terminal you'll see that files have an extension '\*.so.2.4.8' refering to the opencv version, they should have no extension "2.4.8" (simply end with '\*.so'). Also the path of this files is  “/usr/lib/arm-linux-gnueabihf/” and should be “/usr/lib“. In brief, you should mannually make the next following modifications:


    1.remove each instance “/usr/lib/arm-linux-gnueabihf/libopencv_ocl.so.2.4.8;“
    2.replace each instance of “/usr/lib/arm-linux-gnueabihf/” with “/usr/lib“
    3.replace each instance of “2.4.8” with “2.4.12” (or the current version of OpenCV in opencv4tegra package) or simply remove the instance "2.4.8"

These changes ought to be applied in next files:

    sudo gedit lib/pkgconfig/cv_bridge.pc
    sudo gedit lib/pkgconfig/image_geometry.pc
    sudo gedit share/image_geometry/cmake/image_geometryConfig.cmake
    sudo gedit share/cv_bridge/cmake/cv_bridgeConfig.cmake

    
Specific instructions on which files you should make changes to can be found [here](http://myzharbot.robot-home.it/blog/software/ros-nvidia-jetson-tx1-jetson-tk1-opencv-ultimate-guide/). Also you can found a discussion on the error rised when trying to compile without this changes in [here](https://devtalk.nvidia.com/default/topic/900926/how-to-install-the-zed-ros-driver-on-the-jtx1/), and some advices on how to avoid manual modification of files.

### ROS Packages Install 

Install these packages on your Jetson:

    sudo apt-get install ros-indigo-usb-cam
    sudo apt-get install ros-indigo-compressed-image-transport
    sudo apt-get install ros-indigo-image-proc

### Set CUDA paths

If you try to compile now the next error will raise:

    /usr/bin/ld: cannot find -lcudart

To fix these paths to these CUDA dependencies should be changed, instead of these, we'll simple make a simlink of the dependencies, linked to the path where they are looked for. So you have to launch next commands in your terminal:

    sudo ln -s /usr/local/cuda/lib/libcublas.so /usr/lib/libcudart.so
    sudo ln -s /usr/local/cuda/lib/libcublas.so /usr/lib/libcublas.so
    sudo ln -s /usr/local/cuda/lib/libcurand.so /usr/lib/libcurand.so


### Building

In order to install darknet_ros, clone the latest version from this repository into your catkin workspace, switch to the jetson branch and compile the package using ROS inside the Pepper VM.

    cd catkin_workspace/src
    git clone --recursive https://github.com/ReyesDeJong/darknet_ros.git
    git checkout jetson
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

By default tiny-yolo-voc.yalm is set, and there is a cfg and a yalm to an arquitecture that it's lighter than yolo, and can detectect only person class if trained.


## Basic Usage

In order to get YOLO ROS: Real-Time Object Detection for ROS to run with your robot, you will need to adapt a few parameters. It is the easiest if duplicate and adapt all the parameter files that you need to change from the `darkned_ros` package. These are specifically the parameter files in `config` and the launch file from the `launch` folder.

To run the YOLO as it is, using a usb-cam, execute: 

     roslaunch usb_cam usb_cam-test.launch 

open other terminal and execute:

     roslaunch darknet_ros darknet_ros.launch 

## Nodes

### Node: darknet_ros

This is the main YOLO ROS: Real-Time Object Detection for ROS node. It uses the camera measurements to detect pre-learned objects in the frames.

### ROS related parameters

You can change the names and other parameters of the publishers, subscribers and actions inside `darkned_ros/config/ros.yaml`.

#### Subscribed Topics

* **`/camera_reading`** ([sensor_msgs/Image])

    The camera measurements.
    This topic can hear to the camera of Pepper, by setting: `topic: /maqui/camera/front/image_raw`
    or a usb-webcam by setting: `topic: /usb_cam/image_raw`

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

    Keep it true, OpenCV imshow is commented.

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

## Configuration of Jetson with Pepper robot (through a pc connected to Pepper via wi-fi)

![alt tag](https://www.dropbox.com/s/hfc20214x0swrjs/pnj.png?dl=0 "Pepper via wi-fi to notebook, which is connected through ethernet to Jetson.")

After configuring 'ros.yalm' camera reading topic to listen to Pepper's camera, create a ethernet connection with a 'Shared to other computers' method (options found in IPv4 Settings). Then connect Jetson card via ethernet to the computer connected to Pepper. 

### ROS Network Configuration

On a terminal in the Jetson card, type the following:

    export ROS_MASTER_URI=http://192.168.1.129:11311
    ping 192.168.1.129

This is to set Pepper as Master (you have to change 192.168.1.129 to your Pepper IP), and verify to verify a correct connection. Finally, check Jetson's IP of ethernet connection (Go to 'Connection Information' in top-right corner and look for IP Address in the ethernet connection, it should be something like 10.42.0.13) and with this IP execute:

    export ROS_IP=10.42.0.13  

You need to have in mind that every time you open a new terminal, you have to export both IPs.

To check that both IPs are setted, type:

    echo $ROS_IP
    echo $ROS_MASTER_URI

### Launch

Now you can open a terminal in Pepper, connecting to it via ssh (simething like ssh nao@192.168.1.129) and make a full Pepper launch

    roslaunch maqui_bringup maqui.launch 

After, from Jetson card you can launch YOLO

    roslaunch darknet_ros darknet_ros.launch 

And you should se something like:

![alt tag](https://www.dropbox.com/s/dwkicoxz06aknvh/yo.png?dl=0 "YOLO running on Jetson with Pepper camera.")

