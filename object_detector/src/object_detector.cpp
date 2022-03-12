#include "object_detector/object_detector.h"

void ObjectDetector::main() {
  ReadParameters(nh_);

  motion_compensation_.reset(new MotionCompensation);

  img_raw_sub_ = nh_.subscribe(img_raw_topic_, 1, &ObjectDetector::ImgCallback, this);
  events_sub_ = nh_.subscribe(events_topic_, 2, &ObjectDetector::EventsCallback, this);
  imu_sub_ = nh_.subscribe(imu_topic_, 10, &ObjectDetector::ImuCallback, this, ros::TransportHints().tcpNoDelay());
  odom_sub_ = nh_.subscribe(odom_topic_, 10, &ObjectDetector::OdomCallback, this, ros::TransportHints().tcpNoDelay());
}

void ObjectDetector::ReadParameters(ros::NodeHandle &n) {
  n.getParam("/object_detector_node/raw_image_topic", img_raw_topic_);
  n.getParam("/object_detector_node/event_topic", events_topic_);
  n.getParam("/object_detector_node/imu_topic", imu_topic_);
  // n.getParam("/object_detector_node/depth_topic", depth_topic_);
  n.getParam("/object_detector_node/odometry_topic", odom_topic_);
}

void ObjectDetector::ImgCallback(const sensor_msgs::Image::ConstPtr &imgMsg) {

}

void ObjectDetector::EventsCallback(const dvs_msgs::EventArray::ConstPtr &eventMsg) {

}

void ObjectDetector::ImuCallback(const sensor_msgs::ImuConstPtr &imuMsg) {
  motion_compensation_->LoadIMUs(imuMsg);
}

void ObjectDetector::OdomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg) {
  motion_compensation_->LoadOdometry(odomMsg);
}