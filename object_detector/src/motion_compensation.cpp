#include <ros/ros.h>
#include "object_detector/motion_compensation.h"

inline bool MotionCompensation::IsWithinTheBoundary(const int &x, const int &y) {
  return (x >= 0 && x < IMG_COLS && y >= 0 && y < IMG_ROWS);
}

void MotionCompensation::MotionCompensate() {
    ClearData();
    AvgIMU();
    IMU_buffer_.clear();
    RotationalCompensation(&time_img_, &event_counts_);
    MorphologicalOperation(&compensated_time_img_);
}

void MotionCompensation::ClearData() {
    omega_avg_.setZero();
    events_buffer_.clear();
    imu_size_ = IMU_buffer_.size();
    time_img_ = cv::Mat::zeros(cv::Size(IMG_COLS, IMG_ROWS), CV_32FC1);
    event_counts_ = cv::Mat::zeros(cv::Size(IMG_COLS, IMG_ROWS), CV_8UC1);
}

void MotionCompensation::LoadIMUs(const sensor_msgs::ImuConstPtr &imuMsg) {
    IMU_buffer_.push_back(*imuMsg);
}

void MotionCompensation::LoadEvents(const dvs_msgs::EventArray::ConstPtr &eventMsg) {
    events_buffer_.assign(eventMsg->events.begin(), eventMsg->events.end());
    event_size_ = events_buffer_.size();
}

void MotionCompensation::LoadOdometry(const nav_msgs::Odometry::ConstPtr &odomMsg) {
    odoms_buffer_ = *odomMsg;
}

void MotionCompensation::AvgIMU() {
    omega_avg_.setZero();
    imu_size_ = IMU_buffer_.size();
    if (imu_size_ <= 0) {
        omega_ = 0.0f;
    } else {
        for (int i = 0; i < imu_size_; i++) {
            omega_avg_[0] += IMU_buffer_[i].angular_velocity.x;
            omega_avg_[1] += -IMU_buffer_[i].angular_velocity.y;
            omega_avg_[2] += -IMU_buffer_[i].angular_velocity.z;
        }
        omega_avg_ = omega_avg_ / static_cast<float>(imu_size_);
        omega_ = omega_avg_.norm();
    }
}

void MotionCompensation::AccumulateEvents(cv::Mat *timeImg, cv::Mat *eventCount){
    auto t0 = events_buffer_[0].ts;
    dvs_msgs::Event e;
    float delta_T = 0;
    int e_x = 0, e_y = 0;

    for (int i = 0; i < event_size_; i++) {
        e = events_buffer_[i];
        delta_T = (e.ts - t0).toSec();
        e_x = e.x;
        e_y = e.y;

        if (!IsWithinTheBoundary(e_x, e_y)) {
            continue;
        } else {
            int *c = eventCount->ptr<int>(e_y, e_x);
            float *q = timeImg->ptr<float>(e_y, e_x);
            *c += 1;
            *q += (delta_T - *q) / (*c);
        }
    }
}

void MotionCompensation::RotationalCompensation(cv::Mat *timeImg, cv::Mat *eventCount) {
    Eigen::Vector3f rotation_vector;
    Eigen::Vector3f event_vector;
    Eigen::Matrix3f rot_K;
    Eigen::Matrix3f rot_skew_matrix;

    auto t0 = events_buffer_[0].ts;
    float pre_delta_T = 0.0f;
    float delta_T = 0.0f;
    int discretized_x = 0;
    int discretized_y = 0;
    dvs_msgs::Event e;

    for (int i = 0; i < event_size_; i++) {
        e = events_buffer_[i];
        delta_T = (e.ts - t0).toSec();

        /* prepare rotation matrix */
        if (delta_T - pre_delta_T > 1e-3) {
            pre_delta_T = delta_T;
            rotation_vector = omega_avg_ * delta_T;
            rot_skew_matrix = Vector2SkewMatrix(rotation_vector);
            rotation_matrix_ = rot_skew_matrix.exp();
            rot_K = event_camera_K_ * rotation_matrix_.transpose() * event_caemra_K_inverse_;
        }

        /* prepare event vector */
        event_vector[0] = e.x;
        event_vector[1] = e.y;
        event_vector[2] = 1;
        event_vector = rot_K * event_vector;    // warping
        ConvertToHomogeneous(event_vector);

        discretized_x = static_cast<int>(event_vector[0]);
        discretized_y = static_cast<int>(event_vector[1]);

        if (IsWithinTheBoundary(discretized_x, discretized_y)) {
            int *c = eventCount->ptr<int>(discretized_y, discretized_x);
            float *q = timeImg->ptr<float>(discretized_y, discretized_x);
            *c += 1;
            *q += (delta_T - *q) / (*c);
        }
    }
}

void MotionCompensation::MorphologicalOperation(cv::Mat *timeImg) {
    cv::Mat threshold_img;
    cv::Mat tmp_img;
    cv::Mat normalized_time_img = cv::Mat::zeros(cv::Size(IMG_COLS, IMG_ROWS), CV_32FC1);
    cv::normalize(time_img_, normalized_time_img, 0, 1, cv::NORM_MINMAX);

    float threshold = cv::mean(normalized_time_img, event_counts_)[0] +
                        threshold_a_ * omega_ + threshold_b_;

    cv::threshold(normalized_time_img, threshold_img, threshold, 1, cv::THRESH_TOZERO);

    /* Gaussian Blur */
    cv::blur(threshold_img, tmp_img, cv::Size(5, 5));
    cv::normalize(tmp_img, tmp_img, 0, 255, cv::NORM_MINMAX);

    /* Morphological Operation */
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(kernel_size_, kernel_size_),
        cv::Point(-1, -1));
    cv::morphologyEx(tmp_img, tmp_img, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);

    /* element-wise square to enhance the img contrast */
    tmp_img = tmp_img.mul(tmp_img);
    cv::normalize(tmp_img, tmp_img, 0, 255, cv::NORM_MINMAX);
    tmp_img.convertTo(*timeImg, CV_8UC1);
}

void MotionCompensation::VisualizeEventImg(const cv::Mat eventImg) {
    cv::Mat tmp_img, display_img;
    cv::normalize(eventImg, tmp_img, 0, 255, cv::NORM_MINMAX);
    tmp_img.convertTo(tmp_img, CV_8UC1);
    cv::applyColorMap(tmp_img, display_img, cv::COLORMAP_JET);

    cv::namedWindow("Time Image");
    cv::imshow( "window", display_img);
    cv::waitKey(0);
}

Eigen::Matrix3f MotionCompensation::Vector2SkewMatrix(Eigen::Vector3f v) {
    Eigen::Matrix3f skew_matrix;
    skew_matrix << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
    return skew_matrix;
}

void MotionCompensation::ConvertToHomogeneous(Eigen::Vector3f &v) {
    v[0] = v[0] / v[2];
    v[1] = v[1] / v[2];
    v[2] = 1;
}