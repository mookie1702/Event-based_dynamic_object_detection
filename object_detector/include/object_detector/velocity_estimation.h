#ifndef VELOCITY_ESTIMATION_H
#define VELOCITY_ESTIMATION_H

#include <iostream>
#include <memory>

#include <opencv2/opencv.hpp>

#include "dbscan.h"

using namespace std;

class VelocityEstimation {
private:
    /* flags */

    /* data */
    int object_size_;
    vector<DataPoint> data_set_;
    cv::Mat depth_img_;

    vector<cv::Point2f> center_point_;
    ushort object_depth_;

public:
    typedef std::unique_ptr<VelocityEstimation> Ptr;

    VelocityEstimation() {}
    ~VelocityEstimation() {}

    void LoadObjectData(const int object_size, const vector<DataPoint> data_set);
    void LoadDepthImg(const cv::Mat &depth_img);

    void EstimateVelocity();

    void FindCenterPoint();
    void GetObjectDepth();
};

#endif