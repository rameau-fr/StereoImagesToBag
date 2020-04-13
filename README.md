# BagFromImages

Modification of the package BagFromImages for synchronized stereo data and pose. 
I develloped this node for ROS, specifically to convert KITTI odometry into rosbag.

ROS package to generate a rosbag from a collection of images. Images are ordered alphabetically. The timestamp for each image is assigned according to the specified frequency. 

The bag will publish the left images to topic `/camera_left/image_raw`.
The bag will publish the right images to topic `/camera_right/image_raw`.
The bag will publish the pose `/Pose`. The poses are stored under the message format geometry_msgs/Pose

Tested in ROS Kinetic + OpenCV 3.2 + C++ 11

## Installation

Clone in your catking envirronment and compile!

## Usage:

	rosrun BagFromImages stereo_ImageToBag PATH_TO_LEFT_IMAGES PATH_TO_RIGHT_IMAGES PATH_TO_POSE FREQUENCY PATH_TO_OUPUT_BAG
  
 - `PATH_TO_LEFT_IMAGES`: Path to the file containing the sequence of left images (for instance: /dataset/KITTIDATA/sequences/00/image_0)
 - `PATH_TO_RIGHT_IMAGES`: Path to the file containing the sequence of right images (for instance: /dataset/KITTIDATA/sequences/00/image_1)
 - `PATH_TO_POSE`: Path to the pose txt file (KITTI format, /dataset/KITTIDATA/poses/00.txt)
 - `FREQUENCY`: Frames per second.
 - `PATH_TO_OUTPUT_BAG`: Path to save the bag (including the filename e.g. directory/filename.bag)

