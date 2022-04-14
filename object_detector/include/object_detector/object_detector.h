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

using namespace std;


class ObjectDetector {
private:
    /* flags */
    bool is_simulation_;

    /* parameters */
    string k_running_environment;
    string k_img_raw_topic_;
    string k_events_topic_;
    string k_imu_topic_;
    string k_odom_topic_;

    /* detecting utilities */
    MotionCompensation::Ptr motion_compensation_;
    ObjectSegmentation::Ptr object_segmentation_;

    /* ROS utilities */
    ros::NodeHandle &nh_;

    ros::Subscriber img_raw_sub_;
    ros::Subscriber events_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber odom_sub_;

    /* ROS functions */
    void ReadParameters(ros::NodeHandle &n);
    void ImgCallback(const sensor_msgs::Image::ConstPtr &img_msg);
    void EventsCallback(const dvs_msgs::EventArray::ConstPtr &events_msg);
    void ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);

    /* inline functions */

public:
    ObjectDetector(ros::NodeHandle &nh) : nh_(nh) {

    }
    ~ObjectDetector() {}

    void main();
};

#endif