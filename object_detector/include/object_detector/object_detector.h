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
    string events_topic_;
    string imu_topic_;
    // string depth_topic_;
    string odom_topic_;

    /* detecting utilities */
    MotionCompensation::Ptr motion_compensation_;

    /* ROS utilities */
    ros::NodeHandle &nh_;

    ros::Subscriber img_raw_sub_;
    ros::Subscriber events_sub_;
    ros::Subscriber imu_sub_;
    // ros::Subscriber depth_sub_;
    ros::Subscriber odom_sub_;

    /* ROS functions */
    void ReadParameters(ros::NodeHandle &n);
    void ImgCallback(const sensor_msgs::Image::ConstPtr &imgMsg);
    void EventsCallback(const dvs_msgs::EventArray::ConstPtr &eventMsg);
    void ImuCallback(const sensor_msgs::ImuConstPtr &imuMsg);
    // void DepthCallback(const sensor_msgs::ImageConstPtr &depthMsg);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg);

    /* inline functions */

public:
    ObjectDetector(ros::NodeHandle &nh) : nh_(nh) {

    }
    ~ObjectDetector() {}

    void main();
};

#endif