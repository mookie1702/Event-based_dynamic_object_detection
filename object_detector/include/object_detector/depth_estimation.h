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
    /* data */
    cv::Mat depth_gray_ = cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_32FC1);
    geometry_msgs::PointStamped depth_point_;
public:
    typedef std::unique_ptr<DepthEstimation> Ptr;

    DepthEstimation() {}
    ~DepthEstimation() {}

    void EstimateDepth(const sensor_msgs::ImageConstPtr& msg);
};

#endif