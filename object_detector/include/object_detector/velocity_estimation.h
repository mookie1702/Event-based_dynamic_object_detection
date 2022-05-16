#ifndef VELOCITY_ESTIMATION_H
#define VELOCITY_ESTIMATION_H

#include <iostream>
#include <memory>

#include <opencv2/opencv.hpp>

using namespace std;

class VelocityEstimation {
private:
    /* flags */

    /* data */
    cv::Mat depth_img_;

public:
    typedef std::unique_ptr<VelocityEstimation> Ptr;

    VelocityEstimation() {}
    ~VelocityEstimation() {}

    void LoadDepthImg();
    void LoadObjectData();

    void EstimateVelocity();
};

#endif