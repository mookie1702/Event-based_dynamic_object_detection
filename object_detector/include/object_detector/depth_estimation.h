#ifndef DEPTH_ESTIMATION_H
#define DEPTH_ESTIMATION_H

#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/rgbd.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/PointStamped.h"

#define MAT_ROWS 480
#define MAT_COLS 640

using namespace std;

class DepthEstimation {
private:
    /* parameters */
    const int k_valid_frame_ = 10;
    const cv::Mat k_depth_camera_intrinsic_ = (cv::Mat_<float>(3, 3) << 385.7481384277344, 0.0, 319.36944580078125,
                                               0.0, 385.7481384277344, 238.4856414794922,
                                               0.0, 0.0, 1.0);
    const cv::Mat k_event_camera_intrinsic_ = (cv::Mat_<float>(3, 3) << 5.3633325932983780e+02, 0, 3.2090009280822994e+02,
                                               0, 5.3631797700847164e+02, 2.3404853514480661e+02,
                                               0, 0, 1);
    const cv::Mat k_RT_event2depth_ = (cv::Mat_<float>(4, 4) << 1, 0, 0, 0.015,
                                       0, 1, 0, -0.17,
                                       0, 0, 1, 0.0779,
                                       0, 0, 0, 1);
    const cv::Mat k_distort_coeff_ = (cv::Mat_<float>(5, 1) << 0, 0, 0, 0, 0);
    const cv::Size k_event_camera_plane_ = cv::Size(MAT_COLS, MAT_ROWS);

    /* data */
    cv::Mat depth_gray_;
    geometry_msgs::PointStamped depth_point_;
public:
    typedef std::unique_ptr<DepthEstimation> Ptr;

    DepthEstimation() {}
    ~DepthEstimation() {}

    void EstimateDepth(const sensor_msgs::ImageConstPtr& depth_msg);

    cv::Mat GetDepthImg() { return depth_gray_; }
};

#endif