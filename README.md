# ROSbag to Video

This tool allows you to convert a camera topic from a ROSbag into an MP4 video.  You may also consider using the ROS recommended method for [exporting image and video data from a bag file](http://wiki.ros.org/rosbag/Tutorials/Exporting%20image%20and%20video%20data)

## Getting Started

### Pre-requisites

This software was built using ROS Kinetic on Xubuntu 16 LTS on a Chromebook.  ROS installation instructions can be found [here](http://wiki.ros.org/kinetic/Installation/Ubuntu).  You will also need OpenCV and Boost eg:
```bash
sudo apt-get install libopencv-highgui-dev libopencv-dev
```

### Building Software

This tool can be built using standalone ROS (ie it is not a ROS node or service and does not need to be built using ```catkin_make```).  To build:

1. Source your ROS setup file and update paths eg:
```bash
source /opt/ros/kinetic/setup.sh
export CPATH=/opt/ros/kinetic/include:$CPATH
export PKG_CONFIG_PATH=/opt/ros/kinetic/lib/pkgconfig:$PKG_CONFIG_PATH
```
2. Build program with ```make```

### Usage

Once the tool is successfully built, you can run it as:
```bash
rosbag_to_video bag_path topic_name
```

The two required arguments are:
* ```bag_path```: Path of the bag file containing an image topic
* ```topic_name```: Name of the ROS topic you wish to extract into a video eg /camera/image

## Known Limitations & Future Work

* Program writes image messages to disk before converting saved images to a video.  This 2 step process reduces RAM requirements (particularly for larger bag files) at the cost of processing time
* There is currently no support for variable frame rates.  The output video will have the average frame rate of the ROS image messages
  * Update frame rate calculation to use median instead of average to minimize effect of outliers
* Extract all image topics in a ROSbag into individual videos automatically
* No unit tests
* Expects color images

## Sources

Great code makes use of existing resources.  This work leverages prior public software contributions and tutorials:

* [ROSbag API](http://wiki.ros.org/rosbag/Code%20API)
* [Using CvBridge to Convert Between ROS Images and OpenCV Images](http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages)
* [Using ROS software in C++ and Python via CMake and make](https://github.com/gerkey/ros1_external_use)

