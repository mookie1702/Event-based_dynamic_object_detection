#include "object_detector.h"

void ObjectDetector::main() {
  ReadParameters(nh_);

  if ("simulation" == k_running_environment_) {
    is_simulation_ = true;
  } else {
    is_simulation_ = false;
  }

  motion_compensation_.reset(new MotionCompensation(is_simulation_));
  object_segmentation_.reset(new ObjectSegmentation());

  event_sub_ = nh_.subscribe(k_event_topic_, 2, &ObjectDetector::EventCallback, this);
  imu_sub_ = nh_.subscribe(k_imu_topic_, 10, &ObjectDetector::ImuCallback, this, ros::TransportHints().tcpNoDelay());
  odom_sub_ = nh_.subscribe(k_odom_topic_, 10, &ObjectDetector::OdomCallback, this, ros::TransportHints().tcpNoDelay());
}

void ObjectDetector::ReadParameters(ros::NodeHandle &n) {
  n.getParam("/object_detector_node/running_environment", k_running_environment_);
  n.getParam("/object_detector_node/event_topic", k_event_topic_);
  n.getParam("/object_detector_node/imu_topic", k_imu_topic_);
  n.getParam("/object_detector_node/odometry_topic", k_odom_topic_);
}

void ObjectDetector::EventCallback(const dvs_msgs::EventArray::ConstPtr &event_msg) {
  motion_compensation_->LoadEvent(event_msg);

  // Motion Compensation
  motion_compensation_->MotionCompensate();

  /* detect objects base on compensated img */
  object_segmentation_->LoadImg(motion_compensation_->GetEventCount(),
                                motion_compensation_->GetCompensatedTimeImg());
  object_segmentation_->ObjectSegment();
}

void ObjectDetector::ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg) {
  motion_compensation_->LoadIMU(imu_msg);
}

void ObjectDetector::OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
  motion_compensation_->LoadOdometry(odom_msg);
}
