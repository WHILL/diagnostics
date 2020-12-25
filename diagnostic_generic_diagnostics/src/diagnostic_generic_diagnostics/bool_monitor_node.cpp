#include "diagnostic_generic_diagnostics/bool_monitor.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bool_monitor");

  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");

  diagnostic_generic_diagnostics::BoolMonitor bool_monitor(nh, pnh);

  ROS_DEBUG("spinning...");
  ros::spin();
  ROS_DEBUG("exit...");
  return 0;
}