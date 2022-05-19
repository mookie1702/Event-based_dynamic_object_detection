#include "velocity_estimation.h"

int main(int argc, char **argv) {
  ROS_INFO("The velocity estimation node has started!");

  ros::init(argc, argv, "velocity_estimation_node");
  ros::NodeHandle nh("~");

  VelocityEstimation velocity_estimation(nh);
  velocity_estimation.main();
  ros::spin();

  return 0;
}
