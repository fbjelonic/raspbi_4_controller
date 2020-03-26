#pragma once

#include <ros/ros.h>
#include <vector>
#include "std_msgs/UInt16.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "wiringPi.h"
#include "wiringPiI2C.h"
#include "softPwm.h"

#define DEVICE_ID 0x08

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
    void setupI2C();
    void sendMessage(const std::vector<int> &msg);

    int msgStream_;

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
