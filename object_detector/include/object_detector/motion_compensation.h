#ifndef MOTION_COMPENSATION_H
#define MOTION_COMPENSATION_H

#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "dvs_msgs/Event.h"
#include "dvs_msgs/EventArray.h"

using namespace std;

#define IMG_ROWS 480
#define IMG_COLS 640

// #define IMU_BASED
#define OPTIMIZATION

typedef struct M_G_ {
    float h_x_ = 0.0f;
    float h_y_ = 0.0f;
    float h_z_ = 0.0f;
    float theta_ = 0.0f;
}WarpParameters;

class MotionCompensation {
private:
    /* parameters */
    Eigen::Matrix3f k_event_camera_K_;  // event camera's instrinstic matrix
    Eigen::Matrix3f k_event_camera_K_inverse_;

    // threshold parameters
    const float threshold_a_ = 0.2f;
    const float threshold_b_ = -0.1f;
    float omega_ = 0.0f;

    // Mophology operation parameters
    const int kernel_size_ = 3;

    /* data */
    vector<sensor_msgs::Imu> IMU_buffer_;
    vector<dvs_msgs::Event> event_buffer_;
    vector<dvs_msgs::Event> warped_event_buffer_;
    nav_msgs::Odometry odom_buffer_;

    cv::Mat source_time_frame_;
    cv::Mat source_event_count_;
    cv::Mat time_img_;
    cv::Mat event_count_;
    cv::Mat compensated_time_img_;

    Eigen::Matrix3f rotation_matrix_;
    Eigen::Vector3f omega_avg_;

    int event_size_ = 0;
    int imu_size_ = 0;

    // parameters of optimization method
    WarpParameters Prev_M_G_, M_G_;
    const float k_xi_ = 1.0f;

public:
    typedef std::unique_ptr<MotionCompensation> Ptr;

    MotionCompensation(bool is_simulation){
        if (is_simulation) {
            k_event_camera_K_ << 2.5393636730954148e+02, 0.0, 3.205e+02,
                                 0.0, 2.5393636730954148e+02, 2.405e+02,
                                 0.0, 0.0, 1.0;
            k_event_camera_K_inverse_ = k_event_camera_K_.inverse();
        } else {
            k_event_camera_K_ << 5.3633325932983780e+02, 0, 3.2090009280822994e+02,
                                 0, 5.3631797700847164e+02, 2.3404853514480661e+02,
                                 0, 0, 1;
            k_event_camera_K_inverse_ = k_event_camera_K_.inverse();
        }
    }
    ~MotionCompensation(){}

    void MotionCompensate();

    void CleanTimeImgAndEventCount();
    void LoadIMU(const sensor_msgs::ImuConstPtr &imu_msg);
    void LoadEvent(const dvs_msgs::EventArray::ConstPtr &event_msg);
    void LoadOdometry(const nav_msgs::Odometry::ConstPtr &odom_msg);
    void AccumulateEvents(cv::Mat *time_img, cv::Mat *event_count);

    // IMU-based method
    void GetAvgAngularVelocity();
    void RotationalCompensation(cv::Mat *time_img, cv::Mat *event_count);
    void MorphologicalOperation(cv::Mat *time_img);

    // optimized method
    void WarpEvent();
    void GetTimeImg();
    void UpdateModel();
    void Optimization();

    /* display the effect of motion compensation */
    void Visualization(const cv::Mat img, const string window_name);

    cv::Mat GetSourceTimeFrame() { return source_time_frame_; }
    cv::Mat GetSourceEventCount() { return source_event_count_; }
    cv::Mat GetTimeImage() { return time_img_; }
    cv::Mat GetEventCount() { return event_count_; }
    cv::Mat GetCompensatedTimeImg() { return compensated_time_img_; }

    /* inline functions */
    inline bool IsWithinTheBoundary(const int &x, const int &y);

    /* self-defined math functions */
    Eigen::Matrix3f Vector2SkewMatrix(Eigen::Vector3f v);
    void ConvertToHomogeneous(Eigen::Vector3f &v);
};

#endif