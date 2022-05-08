#include "depth_estimation.h"

void DepthEstimation::EstimateDepth(const sensor_msgs::ImageConstPtr& depth_msg) {
    cv_bridge::CvImageConstPtr depth_msg_ptr;
    depth_msg_ptr = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);

    depth_gray_ = cv::Mat::zeros(cv::Size(depth_msg->height, depth_msg->width), CV_8UC1);

    cv::rgbd::registerDepth(k_depth_camera_intrinsic_, k_event_camera_intrinsic_,
                            k_distort_coeff_, k_RT_event2depth_, depth_msg_ptr->image,
                            k_event_camera_plane_, depth_gray_, false);

    /* Morphology operations */
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5), cv::Point(-1, -1));
    morphologyEx(depth_gray_, depth_gray_, CV_MOP_CLOSE, kernel, cv::Point(-1, -1), 1);
    cv::Mat depth_gray_u8(depth_msg->height, depth_msg->width, CV_8UC1);
    depth_gray_.convertTo(depth_gray_u8, CV_8UC1, 1.0 / 256);

}