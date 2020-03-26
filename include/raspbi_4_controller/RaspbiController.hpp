#pragma once

#include <ros/ros.h>
#include "std_msgs/UInt16.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "wiringPi.h"
#include "softPwm.h"

namespace raspbi_controller
{

class RaspbiController
{
public:
    RaspbiController(ros::NodeHandle& nodeHandle);
    virtual ~RaspbiController();
    void servoCallback(const std_msgs::UInt16 &msg);
    void driveCallback(const geometry_msgs::Twist &msg);

private:
    void initPins();

    int sonicServoPin_;
    int leftMotorSpeedPin_;
    int rightMotorSpeedPin_;
    int leftMotorDirPin_;
    int rightMotorDirPin_;
    ros::Subscriber servo_sub_;
    ros::Subscriber drive_sub_;
    ros::NodeHandle nodeHandle_;

};

} // namespace
