#include <ros/ros.h>
#include "object_detector/motion_compensation.h"

inline bool MotionCompensation::IsWithinTheBoundary(const int &x, const int &y) {
  return (x >= 0 && x < IMG_COLS && y >= 0 && y < IMG_ROWS);
}

void MotionCompensation::main() {
    ClearData();
    AvgIMU();
    IMU_buffer_.clear();
    RotationalCompensation(&time_img_, &event_counts_);
}

void MotionCompensation::ClearData() {
    omega_avg_.setZero();
    events_buffer_.clear();
    imu_size_ = IMU_buffer_.size();
    time_img_ = cv::Mat::zeros(cv::Size(IMG_COLS, IMG_ROWS), CV_32FC1);
    event_counts_ = cv::Mat::zeros(cv::Size(IMG_COLS, IMG_ROWS), CV_8UC1);
}

void MotionCompensation::LoadIMUs(const sensor_msgs::ImuConstPtr &imu_msg) {
    IMU_buffer_.push_back(*imu_msg);
}

void MotionCompensation::LoadEvents(const dvs_msgs::EventArray::ConstPtr &event_msg) {
    events_buffer_.assign(event_msg->events.begin(), event_msg->events.end());
    event_size_ = events_buffer_.size();
}

void MotionCompensation::LoadOdometry(const nav_msgs::Odometry::ConstPtr &odom_msg) {
    odoms_buffer_ = *odom_msg;
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

cv::Mat VisualizeEventImg(const cv::Mat timeImg) {
    cv::Mat m, m_color;
    cv::normalize(timeImg, m, 0, 255, cv::NORM_MINMAX);
    m.convertTo(m, CV_8UC1);
    cv::applyColorMap(m, m_color, cv::COLORMAP_JET);
    return m_color;
}

Eigen::Matrix3f MotionCompensation::Vector2SkewMatrix(Eigen::Vector3f v) {
    Eigen::Matrix3f skew_matrix;
    skew_matrix << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
    return skew_matrix;
}

void MotionCompensation::ConvertToHomogeneous(Eigen::Vector3f& v) {
    v[0] = v[0] / v[2];
    v[1] = v[1] / v[2];
    v[2] = 1;
}