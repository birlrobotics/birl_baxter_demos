# YOLO for ROS with Kinect
This package provides an ROS interface to run the YOLO in the ROS as a node that subscribe camera topic,and you can choose to use webcam or Kinect to perform Real-Time Object Detection.The defaoult camera is Kinect.If you want to use another camera,you need to change the subscribed topic name in the yolo_ros.cpp. 
Besides,the package hasn't contains any pretrained convolutional weights,you need to download on the website and default uses the tiny-yolo-voc.weights.Of course,you can use your trained convolutional weights but don't forget to match the object labol in yolo_ros.cpp.

This package has been tested in Ubuntu 14.04 with camera of Kinect and webcam.

## Preparation
If you have installed the usb_cam package,Kinect_ros package and CUDA,you can ignore this part.

1.usb_cam:

`cd ~/catkin_ws/src`

`git clone https://github.com/bosch-ros-pkg/usb_cam.git`

`cd ~/catkin_ws`

`catkin_make`

You can test the webcam after installed the package.

`roslaunch usb_cam usb_cam.launch`

2.Kinect_ros:

`sudo apt-get install ros-indigo-openni-* ros-indigo-openni2-* ros-indigo-freenect-*`

`rospack profile`

You can test the camera after installed the package.

`roscore`

`roslaunch freenect_launch freenect.launch`

show RGB image:

`rosrun image_view image_view image:=/camera/rgb/image_color`

show Depth image:

`rosrun image_view disparity_view image:=/camera/depth/disparity`

3.CUDA 8.0
To install CUDA has a long progress,I recommand to follow the official tutorial.The tutorial weill tell you how to do in detail but need more patient to follow it.

## Installation
Git clone the package into your catkin workspace.

If you had cloned this package you can ignore this step.

`cd catkin_ws/src`

`git clone --recursive https://github.com/Jiajie-Ye/darknet_ros.git`


Then,you need to download the weights file on the website.

`cd catkin_ws/src/darknet_ros/weights/`

`wget https://pjreddie.com/media/files/yolo.weights`

`wget https://pjreddie.com/media/files/tiny-yolo-voc.weights`

And then,often the yolo_ros.cpp file and modify the workspace name.(default workspace name:catkin_ws)

`char *cfg = "/home/user/catkin_ws/src/darknet_ros/cfg/tiny-yolo-voc.cfg";`

`char *weights = "/home/user/catkin_ws/src/darknet_ros/weights/tiny-yolo-voc.weights";`

Last,compile the package.

`cd ../`

`catkin_make`

## RUN
Run the yolo_ros node and camera.

`roslaunch darknet_ros yolo_kinect_ros.launch`

## Use other trained weights files: 
Firstly,you need to provide the weights and cfg files into the directories:

`catkin_ws/src/darknet_ros/weights/`

`catkin_ws/src/darknet_ros/cfg/`

Then,often the yolo_ros.cpp file and modify the current path of the weights and cfg files you have trained.

`char *cfg = "/home/user/catkin_ws/src/darknet_ros/cfg/yolo.cfg";`

`char *weights = "/home/user/catkin_ws/src/darknet_ros/weights/yolo.weights";`

Compile with catkin_make and run it.

## Use other camera: 
In order to use other camer,you need to modify the camera topic name and the launch file.
for example,I want to use the webcam,modify the name just like the following lines ①②③：

`const std::string CAMERA_TOPIC_NAME = "/camera/rgb/image_raw";//1`

`const std::string CAMERA_WIDTH_PARAM ="/yolo_ros/image_width";//2`

`const std::string CAMERA_HEIGHT_PARAM ="/yolo_ros/image_height";//3`

`//const std::string CAMERA_TOPIC_NAME = "/usb_cam/image_raw";//①`

`//const std::string CAMERA_WIDTH_PARAM = "/usb_cam/image_width";//②`

`//const std::string CAMERA_HEIGHT_PARAM = "/usb_cam/image_height";//③`


Then,run the following command:

`roslaunch darknet_ros yolo_webcam_ros.launch`

## Issues
Q1:When catkin_make the package it shows a warning like that `warning:deprecated conversion from string constant to ‘char*’` ,and it also can compile successfully,but it won't run after run the launch file.
A1:modify the define of cfg and weights as the followings:

`char *cfg =(char*) "/home/user/catkin_ws/src/darknet_ros/cfg/yolo.cfg";`

`char *weights =(char*) "/home/user/catkin_ws/src/darknet_ros/weights/yolo.weights";`

Compile with catkin_make and run it.

