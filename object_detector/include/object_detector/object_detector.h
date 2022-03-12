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
#include "clustering.h"

using namespace std;

class ObjectDetector {
private:
    /* flags */

    /* parameters */
    string img_raw_topic_;
    string event_camera_topic_;
    string imu_topic_;
    string depth_camera_topic_;
    string odom_topic_;

    /* detecting utilities */
    MotionCompensation::Ptr motion_compensation_;

    /* ROS utilities */
    ros::NodeHandle &nh_;

    ros::Subscriber img_raw_sub_;
    ros::Subscriber event_camera_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber depth_camera_sub_;
    ros::Subscriber odom_sub_;

    /* ROS functions */
    void ReadParameters(ros::NodeHandle &n);
    void ImageCallback(const sensor_msgs::Image::ConstPtr &imsg);
    void EventsCallback(const dvs_msgs::EventArray::ConstPtr &emsg);
    void ImuCallback(const sensor_msgs::ImuConstPtr &imu);
    void DepthCallback(const sensor_msgs::ImageConstPtr &msg);
    void OdometryCallback(const nav_msgs::Odometry::ConstPtr &odom);

    /* inline functions */

public:
    ObjectDetector(/* args */);
    ~ObjectDetector();

    void main();
};

#endif