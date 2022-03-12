#include "object_detector/object_detector.h"

void ObjectDetector::main() {

}

void ObjectDetector::ReadParameters(ros::NodeHandle &n) {
  n.getParam("/object_detector_node/depth_topic", depth_camera_topic_);
  n.getParam("/object_detector_node/imu_topic", imu_topic_);
  n.getParam("/object_detector_node/raw_image_topic", img_raw_topic_);
  n.getParam("/object_detector_node/event_topic", event_camera_topic_);
  n.getParam("/object_detector_node/odometry_topic", odom_topic_);
}