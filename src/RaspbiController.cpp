#include "raspbi_4_controller/RaspbiController.hpp"

namespace raspbi_controller {

RaspbiController::RaspbiController(ros::NodeHandle& nodeHandle):
    nodeHandle_(nodeHandle)
{
    servo_sub_ = nodeHandle_.subscribe("/servo", 10, &RaspbiController::servoCallback ,this);
    drive_sub_ = nodeHandle_.subscribe("/drive", 10, &RaspbiController::driveCallback, this);
}

RaspbiController::~RaspbiController()
{
}

void RaspbiController::servoCallback(const std_msgs::UInt16 &msg)
{
    ROS_INFO("Got the message: Twist Sensor to %u Degrees", msg.data);
}

void RaspbiController::driveCallback(const geometry_msgs::Twist &msg)
{
    int left_wheel = static_cast<int>(msg.linear.x - msg.angular.z);
    int right_wheel = static_cast<int>(msg.linear.x + msg.angular.z);
    ROS_INFO("Got the message. Left wheel to %d and right wheel to %d", left_wheel, right_wheel);
}

} // namespace
