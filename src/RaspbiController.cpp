#include "raspbi_4_controller/RaspbiController.hpp"

namespace raspbi_controller {

RaspbiController::RaspbiController(ros::NodeHandle& nodeHandle):
    nodeHandle_(nodeHandle)
{
    setupI2C();
    initPins();
    servo_sub_ = nodeHandle_.subscribe("/servo", 10, &RaspbiController::servoCallback ,this);
    drive_sub_ = nodeHandle_.subscribe("/drive", 10, &RaspbiController::driveCallback, this);
}

void RaspbiController::setupI2C()
{
    msgStream_ = wiringPiI2CSetup(DEVICE_ID);
    if (msgStream_ == -1) {
        ROS_ERROR("Failed to init I2C communication.");
        ros::shutdown();
    }
    ROS_INFO("Successfully init I2C communication with arduino!");
}

void RaspbiController::initPins()
{
    wiringPiSetupGpio();
    // ultrasonic servo
    sonicServoPin_ = 13;
    softPwmCreate(sonicServoPin_, 0, 180);

    // diffdrive
    leftMotorSpeedPin_ = 11; // pwm speed output for left wheel
    rightMotorSpeedPin_ = 12; // pwm speed output for right wheel
    leftMotorDirPin_ = 15; // HIGH = forward, LOW = backward
    rightMotorDirPin_ = 16; // HIGH = forward, LOW = backward
    softPwmCreate(leftMotorSpeedPin_, 0, 255); // 255 = full speed
    softPwmCreate(rightMotorSpeedPin_, 0, 255); // 255 = full speed
    pinMode(leftMotorDirPin_, OUTPUT);
    pinMode(rightMotorDirPin_, OUTPUT);
}

RaspbiController::~RaspbiController()
{
}

void RaspbiController::servoCallback(const std_msgs::UInt16 &msg)
{
    wiringPiI2CWrite(msgStream_, msg.data);
    ROS_INFO("Got the message: Twist Sensor to %u Degrees", msg.data);
}

void RaspbiController::driveCallback(const geometry_msgs::Twist &msg)
{
    int left_wheel = static_cast<int>(msg.linear.x - msg.angular.z);
    int left_dir = (left_wheel >=0);
    int right_wheel = static_cast<int>(msg.linear.x + msg.angular.z);
    int right_dir = (right_wheel >=0);
    if (left_wheel > 255) {
        left_wheel = 255;
    }
    else if (left_wheel < -255) {
        left_wheel = -255;
    }
    if (right_wheel > 255) {
        right_wheel = 255;
    }
    else if (right_wheel < -255) {
        right_wheel = -255;
    }
//    softPwmWrite(leftMotorSpeedPin_, left_wheel);
//    softPwmWrite(rightMotorSpeedPin_, right_wheel);
//    digitalWrite(leftMotorDirPin_, left_dir);
//    digitalWrite(rightMotorDirPin_, right_dir);
//    ROS_INFO("Got the message. Left wheel to %d and right wheel to %d", left_wheel, right_wheel);
}

} // namespace
