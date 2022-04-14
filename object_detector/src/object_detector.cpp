#include "object_detector.h"

void ObjectDetector::main() {
  ReadParameters(nh_);
  if ("simulation" == k_running_environment) {
    is_simulation_ = true;
  } else {
    is_simulation_ = false;
  }

  motion_compensation_.reset(new MotionCompensation(is_simulation_));

  img_raw_sub_ = nh_.subscribe(k_img_raw_topic_, 1, &ObjectDetector::ImgCallback, this);
  events_sub_ = nh_.subscribe(k_events_topic_, 2, &ObjectDetector::EventsCallback, this);
  imu_sub_ = nh_.subscribe(k_imu_topic_, 10, &ObjectDetector::ImuCallback, this, ros::TransportHints().tcpNoDelay());
  odom_sub_ = nh_.subscribe(k_odom_topic_, 10, &ObjectDetector::OdomCallback, this, ros::TransportHints().tcpNoDelay());
}

void ObjectDetector::ReadParameters(ros::NodeHandle &n) {
  n.getParam("/object_detector_node/running_environment", k_running_environment);
  n.getParam("/object_detector_node/raw_image_topic", k_img_raw_topic_);
  n.getParam("/object_detector_node/event_topic", k_events_topic_);
  n.getParam("/object_detector_node/imu_topic", k_imu_topic_);
  n.getParam("/object_detector_node/odometry_topic", k_odom_topic_);
}

void ObjectDetector::ImgCallback(const sensor_msgs::Image::ConstPtr &img_msg) {

}

void ObjectDetector::EventsCallback(const dvs_msgs::EventArray::ConstPtr &event_msg) {
  /* ego-motion compensation */
  motion_compensation_->LoadEvents(event_msg);
  motion_compensation_->MotionCompensate();

  /* detect objects base on compensated img */
  // cv::Mat compensated_time_img_, event_count_;
  // compensated_time_img_ = motion_compensation_->GetCompensatedTimeImg();
  // event_count_ = motion_compensation_->GetEventCount();
}

void ObjectDetector::ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg) {
  motion_compensation_->LoadIMUs(imu_msg);
}

void ObjectDetector::OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
  motion_compensation_->LoadOdometry(odom_msg);
}