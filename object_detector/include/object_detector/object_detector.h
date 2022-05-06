#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <ctime>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

#include "motion_compensation.h"
#include "object_segmentation.h"
#include "depth_estimation.h"

using namespace std;

class ObjectDetector {
private:
    /* flags */
    bool is_simulation_;

    /* parameters */
    string k_running_environment_;
    string k_event_topic_;
    string k_imu_topic_;
    string k_depth_topic_;

    /* detecting utilities */
    MotionCompensation::Ptr motion_compensation_;
    ObjectSegmentation::Ptr object_segmentation_;
    DepthEstimation::Ptr depth_estimation_;

    /* ROS utilities */
    ros::NodeHandle &nh_;
    ros::Subscriber event_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber depth_sub_;

    /* ROS functions */
    void ReadParameters(ros::NodeHandle &n);
    void EventCallback(const dvs_msgs::EventArray::ConstPtr &event_msg);
    void ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg);
    void DepthCallback(const sensor_msgs::ImageConstPtr &msg);

public:
    ObjectDetector(ros::NodeHandle &nh) : nh_(nh) {}
    ~ObjectDetector() {}

    void main();
};

#endif