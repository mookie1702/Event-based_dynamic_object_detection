#ifndef VELOCITY_ESTIMATION_H
#define VELOCITY_ESTIMATION_H

#include <iostream>
#include <memory>

#include <opencv2/opencv.hpp>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "dbscan.h"

using namespace std;

class VelocityEstimation {
private:
    /* data */
    int object_size_;
    vector<DataPoint> data_set_;
    geometry_msgs::PointStamped object_point_in_event_;
    cv::Mat depth_img_;

public:
    typedef std::unique_ptr<VelocityEstimation> Ptr;

    VelocityEstimation() {}
    ~VelocityEstimation() {}

    void EstimateVelocity();

    void LoadObjectData(const int object_size,
                        const vector<DataPoint> data_set,
                        const geometry_msgs::PointStamped object_point_in_event);
    void LoadDepthImg(const cv::Mat &depth_img);
};

#endif