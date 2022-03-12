#include "object_detector/object_detector.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "object_detector_node");
  ros::NodeHandle nh("~");

  ROS_INFO("The object detector node has started!");
  ros::spin();

  return 0;
}