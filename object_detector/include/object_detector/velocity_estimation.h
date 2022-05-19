#ifndef VELOCITY_ESTIMATION_H
#define VELOCITY_ESTIMATION_H

#include <iostream>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <ctime>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Odometry.h"

using namespace std;

class VelocityEstimation {
private:
    /* flags */

    /* parameters */
    const Eigen::Vector3d k_Grav_{0, 0, -9.81};

    /* ROS utilities */
    ros::NodeHandle &nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber event_point_sub_;
    ros::Subscriber depth_point_sub_;

    /**
     * @brief frame traslation
     * * cam frame: 'z' backwards, 'y' downwards
     * * body frame: 'X' forwards, 'z' upwards
     * * world frame: body frame when initializing
     */
    Eigen::Isometry3d T_c2b_; // cam frame to body frame
    Eigen::Isometry3d T_c2w_; // cam frame to world frame
    Eigen::Isometry3d T_w2c_; // world frame to cam frame
    Eigen::Isometry3d T_b2w_; // body frame to world frame

public:
    typedef std::unique_ptr<VelocityEstimation> Ptr;

    VelocityEstimation(ros::NodeHandle &nh) : nh_(nh) {}
    ~VelocityEstimation() {}

    void main();
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void EventCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
    void DepthCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
};

#endif