#ifndef MOTION_COMPENSATION_H
#define MOTION_COMPENSATION_H

/* INCLUDES */
#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>

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

class MotionCompensation {
private:
    /* flags */

    /* parameters */
    Eigen::Matrix3f event_camera_K_;    // event camera's instrinstic matrix
    Eigen::Matrix3f event_camera_K_inverse_;

    // threshold parameters
    const float threshold_a_ = 0.0;
    const float threshold_b_ = 0.0;
    float omega_ = 0.0f;

    // Mophology operation parameters
    const int kernel_size_ = 3;


    /* data */
    vector<sensor_msgs::Imu> IMU_buffer_;
    vector<dvs_msgs::Event> events_buffer_;
    nav_msgs::Odometry odoms_buffer_;

    // cv::Mat depth_img_;
    cv::Mat source_time_frame_;
    cv::Mat source_event_count_;

    cv::Mat time_img_;     // mean-time image
    cv::Mat event_count_;  // the img that counts events in each pixel

    cv::Mat compensated_time_img_;

    /* utilities */
    Eigen::Matrix3f rotation_matrix_;
    Eigen::Vector3f omega_avg_;

    int event_size_ = 0;
    int imu_size_ = 0;

    /* inline functions */
    inline bool IsWithinTheBoundary(const int &x, const int &y);

public:
    typedef std::unique_ptr<MotionCompensation> Ptr;

    MotionCompensation(){
        event_camera_K_ << 5.3633325932983780e+02, 0, 3.2090009280822994e+02, 0,
            5.3631797700847164e+02, 2.3404853514480661e+02, 0, 0, 1;
        event_camera_K_inverse_ = event_camera_K_.inverse();
    }
    ~MotionCompensation(){}

    void MotionCompensate();

    void ClearData();
    void LoadIMUs(const sensor_msgs::ImuConstPtr &imuMsg);
    void LoadEvents(const dvs_msgs::EventArray::ConstPtr &eventMsg);
    void LoadOdometry(const nav_msgs::Odometry::ConstPtr &odomMsg);
    // void LoadDepth(const cv::Mat &depth);

    void AvgIMU();
    void AccumulateEvents(cv::Mat *timeImg, cv::Mat *eventCount);
    void RotationalCompensation(cv::Mat *timeImg, cv::Mat *eventCount);
    // void TranslationalCompensation(cv::Mat *timeImg, cv::Mat *eventCount);
    // void RotTransCompensation(cv::Mat *timeImg, cv::Mat *eventCount);

    void MorphologicalOperation(cv::Mat *timeImg);

    /* display the effect of motion compensation */
    void Visualization(const cv::Mat eventImg, const string windowName);

    cv::Mat GetSourceTimeFrame(void) { return source_time_frame_; }
    cv::Mat GetSourceEventCount(void) { return source_event_count_; }
    cv::Mat GetTimeImage(void) { return time_img_; }
    cv::Mat GetEventCount(void) { return event_count_; }
    cv::Mat GetCompensatedTimeImg(void) { return compensated_time_img_; }

    /* self-defined math functions */
    Eigen::Matrix3f Vector2SkewMatrix(Eigen::Vector3f v);
    void ConvertToHomogeneous(Eigen::Vector3f &v);
};


#endif  // MOTION_COMENSATION_H