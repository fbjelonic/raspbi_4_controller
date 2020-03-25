#include <ros/ros.h>
#include "raspbi_4_controller/RaspbiController.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "raspbi_controller");
  ros::NodeHandle nodeHandle("~");

  raspbi_controller::RaspbiController raspbiController(nodeHandle);

  ros::spin();
  return 0;
}

