#include "velocity_estimation.h"

void VelocityEstimation::LoadDepthImg(const cv::Mat& depth_img) {
    depth_img_ = depth_img.clone();
}

void VelocityEstimation::LoadObjectData(const int object_size, const vector<DataPoint> data_set) {
    object_size_ = object_size;
    data_set_ = data_set;
}

void VelocityEstimation::EstimateVelocity() {
    FindCenterPoint();
    GetObjectDepth();

    // cv::Mat display_img = depth_img_.clone();
    // cv::circle(display_img, center_point_[0], 10, cv::Scalar(0, 0, 0), 1);
    // cv::imshow("depth_object", display_img);
    // cv::waitKey(0);
}

void VelocityEstimation::FindCenterPoint() {
    cv::Point2f min_point;
    cv::Point2f max_point;
    min_point.x = data_set_[0].x_;
    min_point.y = data_set_[0].y_;
    max_point.x = data_set_[0].x_;
    max_point.y = data_set_[0].y_;

    for (auto point : data_set_) {
        if (max_point.x < point.x_)
            max_point.x = point.x_;
        if (max_point.y < point.y_)
            max_point.y = point.y_;
        if (min_point.x > point.x_)
            min_point.x = point.x_;
        if (min_point.y > point.y_)
            min_point.y = point.y_;
    }

    cv::Point2f center_point;
    center_point.y = (min_point.x + max_point.x) / 2;
    center_point.x = (min_point.y + max_point.y) / 2;
    center_point_.push_back(center_point);
}

void VelocityEstimation::GetObjectDepth() {
    object_depth_ = depth_img_.at<ushort>(center_point_[0].x, center_point_[0].y);
    cout << "the depth of object is: " << object_depth_ << endl;
}
