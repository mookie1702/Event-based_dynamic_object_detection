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
#include "dvs_msgs/Event.h"
#include "dvs_msgs/EventArray.h"

using namespace std;

#define IMG_ROWS 480
#define IMG_COLS 640

#define IMU_BASED
// #define OPTIMIZATION

#ifdef OPTIMIZATION
typedef struct warped_event {
    float x;
    float y;
    float ts;
} WarpedEvent;

typedef struct warp_parameter {
    float h_x = 0.0f;
    float h_y = 0.0f;
    float h_z = 0.0f;
    float theta = 0.0f;
} WarpParameter;
#endif

class MotionCompensation {
private:
    /* parameters */
    Eigen::Matrix3f k_event_camera_K_;
    Eigen::Matrix3f k_event_camera_K_inverse_;

    /* data */
    vector<sensor_msgs::Imu> IMU_buffer_;
    vector<dvs_msgs::Event> event_buffer_;
    int event_size_ = 0;
    int imu_size_ = 0;

    cv::Mat source_time_frame_;
    cv::Mat source_event_count_;
    cv::Mat time_img_;
    cv::Mat event_count_;
    cv::Mat compensated_time_img_;

#ifdef IMU_BASED
    Eigen::Matrix3f rotation_matrix_;
    Eigen::Vector3f omega_avg_;

    // Mophology operation and threshold parameters
    const float threshold_a_ = 0.2f;
    const float threshold_b_ = -0.1f;
    float omega_ = 0.0f;
    const int kernel_size_ = 3;
#endif

#ifdef OPTIMIZATION
    vector<WarpedEvent> warped_event_buffer_;
    int warped_event_size_;
    WarpParameter prev_M_G_, M_G_;
    const float k_d_ = 0.5f;
    const float k_xi_ = 0.00001f;
#endif

public:
    typedef std::unique_ptr<MotionCompensation> Ptr;

    MotionCompensation(bool is_simulation) {
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
    ~MotionCompensation() {}

    void MotionCompensate();

    void CleanTimeImgAndEventCount();
    void LoadIMU(const sensor_msgs::ImuConstPtr &imu_msg);
    void LoadEvent(const dvs_msgs::EventArray::ConstPtr &event_msg);
    void AccumulateEvents(cv::Mat *time_img, cv::Mat *event_count);


#ifdef IMU_BASED
    void GetAvgAngularVelocity();
    void RotationalCompensation(cv::Mat *time_img, cv::Mat *event_count);
    void MorphologicalOperation(cv::Mat *time_img);
#endif

#ifdef OPTIMIZATION
    void CleanWarpParameter();
    void WarpEventCloud(WarpParameter para);
    void GetTimestampImg(cv::Mat *time_img, cv::Mat *event_count);
    void UpdateModel(cv::Mat &time_img);
#endif

    /* display the effect of motion compensation */
    void Visualization(const cv::Mat img, const string window_name);

    cv::Mat GetTimeImage() { return time_img_; }
    cv::Mat GetEventCount() { return event_count_; }
    cv::Mat GetCompensatedTimeImg() { return compensated_time_img_; }

    /* inline functions */
    inline bool IsWithinTheBoundary(const int &x, const int &y, cv::Mat &img);

    /* self-defined math functions */
    Eigen::Matrix3f Vector2SkewMatrix(Eigen::Vector3f v);
    void ConvertToHomogeneous(Eigen::Vector3f &v);
};

#endif