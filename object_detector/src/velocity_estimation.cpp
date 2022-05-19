#include "velocity_estimation.h"

void VelocityEstimation::LoadDepthImg(const cv::Mat& depth_img) {
    depth_img_ = depth_img.clone();
}

void VelocityEstimation::LoadObjectData(const int object_size, const vector<DataPoint> data_set) {
    object_size_ = object_size;
    data_set_ = data_set;
}

void VelocityEstimation::EstimateVelocity() {

    // cv::Mat display_img = depth_img_.clone();
    // cv::circle(display_img, center_point_[0], 10, cv::Scalar(0, 0, 0), 1);
    // cv::imshow("depth_object", display_img);
    // cv::waitKey(0);
}

void VelocityEstimation::GetObjectDepth() {

}
