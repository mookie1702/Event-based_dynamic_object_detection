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
  depth_estimation_.reset(new DepthEstimation());

  // img_sub_ = nh_.subscribe(k_img_topic_, 1, &ObjectDetector::ImgCallback, this);
  event_sub_ = nh_.subscribe(k_event_topic_, 2, &ObjectDetector::EventCallback, this);
  imu_sub_ = nh_.subscribe(k_imu_topic_, 10, &ObjectDetector::ImuCallback, this, ros::TransportHints().tcpNoDelay());
  depth_sub_ = nh_.subscribe(k_depth_topic_, 1, &ObjectDetector::DepthCallback, this, ros::TransportHints().tcpNoDelay());

  object_event_pub = nh_.advertise<geometry_msgs::PointStamped>("/object_event_point", 1);
  object_depth_pub = nh_.advertise<geometry_msgs::PointStamped>("/object_depth_point", 1);
}

void ObjectDetector::ReadParameters(ros::NodeHandle &n) {
  n.getParam("/object_detector_node/running_environment", k_running_environment_);
  n.getParam("/object_detector_node/raw_image_topic", k_img_topic_);
  n.getParam("/object_detector_node/event_topic", k_event_topic_);
  n.getParam("/object_detector_node/imu_topic", k_imu_topic_);
  n.getParam("/object_detector_node/depth_topic", k_depth_topic_);
}

void ObjectDetector::ImgCallback(const sensor_msgs::Image::ConstPtr &img_msg) {
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::TYPE_8UC3);
  cv::Mat display_img = cv_ptr->image;
  cv::imshow("img_raw", display_img);
  cv::waitKey(0);
}

void ObjectDetector::EventCallback(const dvs_msgs::EventArray::ConstPtr &event_msg) {
  motion_compensation_->LoadEvent(event_msg);

  // Motion Compensation
  motion_compensation_->MotionCompensate();

  /* detect objects on compensated img */
  cv::Mat time_img, event_count;
  time_img = motion_compensation_->GetTimeImage();
  event_count = motion_compensation_->GetEventCount();

  object_segmentation_->LoadImg(motion_compensation_->GetEventCount(),
                                motion_compensation_->GetCompensatedTimeImg());
  object_segmentation_->ObjectSegment();

  if (!object_segmentation_->GetIsObject()) {
    depth_estimation_->SetIsObject(object_segmentation_->GetIsObject());
  } else {
    cv::Rect roi_rect = object_segmentation_->GetROIRect();
    cv::Mat roi_mat = time_img(roi_rect);
    roi_mat.convertTo(roi_mat, CV_8U);
    auto ts = cv::mean(roi_mat, roi_mat);

    depth_estimation_->SetIsObject(object_segmentation_->GetIsObject());
    depth_estimation_->SetEventDetectionRect(roi_rect);

    geometry_msgs::PointStamped object_point_in_event;
    object_point_in_event.header.stamp = event_msg->events[0].ts + ros::Duration(ts[0]);
    object_point_in_event.header.frame_id = "/cam";
    object_point_in_event.point.x = roi_rect.x + roi_rect.width * 0.5f;
    object_point_in_event.point.y = roi_rect.y + roi_rect.height * 0.5f;
    object_point_in_event.point.z = 0;
    object_event_pub.publish(object_point_in_event);
  }
}

void ObjectDetector::ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg) {
  motion_compensation_->LoadIMU(imu_msg);
}

void ObjectDetector::DepthCallback(const sensor_msgs::ImageConstPtr &depth_msg) {
  depth_estimation_->EstimateDepth(depth_msg);
  if (depth_estimation_->GetIsObject()) {
    geometry_msgs::PointStamped object_point_in_depth = depth_estimation_->GetDepthPoint();
    object_depth_pub.publish(object_point_in_depth);
  }
}
