#include "object_detector.h"

int main(int argc, char **argv) {
  ROS_INFO("The object detector node has started!");

  ros::init(argc, argv, "object_detector_node");
  ros::NodeHandle nh("~");
  ObjectDetector detector(nh);
  detector.main();
  ros::spin();

  return 0;
}