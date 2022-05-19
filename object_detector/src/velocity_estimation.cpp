#include "velocity_estimation.h"

void VelocityEstimation::main() {
    /* rotation from camera frame to body frame */
    Eigen::Matrix3d R_c2b;
    R_c2b << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    Eigen::Quaterniond q_c2b(R_c2b);
    T_c2b_ = Eigen::Isometry3d::Identity();
    T_c2b_.rotate(q_c2b);

    odom_sub_ = odom_sub_ = nh_.subscribe("/vins_estimator/odometry", 1, &VelocityEstimation::OdomCallback,
                                          this, ros::TransportHints().tcpNoDelay());
    event_point_sub_ = nh_.subscribe("/object_event_point", 10, &VelocityEstimation::EventCallback,
                                     this, ros::TransportHints().tcpNoDelay());
    depth_point_sub_ = nh_.subscribe("/object_depth_point", 10, &VelocityEstimation::DepthCallback,
                                     this, ros::TransportHints().tcpNoDelay());
}

void VelocityEstimation::OdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    Eigen::Quaterniond q;
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    q.w() = msg->pose.pose.orientation.w;

    T_b2w_.setIdentity();
    T_b2w_.rotate(q);
    T_b2w_.pretranslate(Eigen::Vector3d(msg->pose.pose.position.x,
                                        msg->pose.pose.position.y,
                                        msg->pose.pose.position.z));

    T_c2w_ = T_b2w_ * T_c2b_;
    T_w2c_ = T_c2w_.inverse();
}

void VelocityEstimation::EventCallback(const geometry_msgs::PointStamped::ConstPtr &msg) {
    // ros::Time t_now = msg->header.stamp;

    // Eigen::Vector3d p_c;
    // Eigen::Vector2d p_pix(msg->point.x, msg->point.y);
    // cam_->liftProjective(p_pix, p_c);
    // Eigen::Vector3d p_w = T_c2w_ * p_c;

    // geometry_msgs::PointStamped vis;
    // vis.header.stamp = msg->header.stamp;
    // vis.header.frame_id = "/world";
    // vis.point.x = p_w(0);
    // vis.point.y = p_w(1);
    // vis.point.z = p_w(2);
}

void VelocityEstimation::DepthCallback(const geometry_msgs::PointStamped::ConstPtr &msg) {
    ros::Time t_now = msg->header.stamp;

    /* Get position of depth obs in world frame */
    geometry_msgs::PointStamped p;
    Eigen::Vector3d p_cam(msg->point.x, msg->point.y, msg->point.z);
    Eigen::Vector3d p_world = T_c2w_ * p_cam;
    p.header = msg->header;
    p.header.frame_id = "/world";
    // millimeter -> meter
    p.point.x = p_world(0) * 1e-3;
    p.point.y = p_world(1) * 1e-3;
    p.point.z = p_world(2) * 1e-3;

    cout << "The position of object in the world frame is:" << endl;
    cout << "\t" << p.point.x << "\t" << p.point.y << "\t" << p.point.z << endl;
}
