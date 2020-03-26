#include "raspbi_4_controller/RaspbiController.hpp"

namespace raspbi_controller {

RaspbiController::RaspbiController(ros::NodeHandle& nodeHandle):
    nodeHandle_(nodeHandle)
{
    setupI2C();
    // initPins();
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
    std::vector<int> send = {1, 90};
    std::vector<int> send2 {2, 255, 1, 255, -1};
}

void RaspbiController::sendMessage(const std::vector<int> &msg)
{
    for (size_t i=0 ; i < msg.size() ; i++) {
         wiringPiI2CWrite(msgStream_, msg.at(i));
    }
}

RaspbiController::~RaspbiController()
{
}

void RaspbiController::servoCallback(const std_msgs::UInt16 &msg)
{
    // 1 = servo, msg.data = angle
    std::vector<int> send_msg = {1, msg.data};
    sendMessage(send_msg);
    ROS_INFO("Got the message: Twist Sensor to %u Degrees", msg.data);
}

void RaspbiController::driveCallback(const geometry_msgs::Twist &msg)
{
    int left_wheel = static_cast<int>(msg.linear.x - msg.angular.z);
    int left_dir = (left_wheel >= 0);
    int right_wheel = static_cast<int>(msg.linear.x + msg.angular.z);
    int right_dir = (right_wheel >=0);
    ROS_INFO("Got the message. Left wheel to %d and right wheel to %d", left_wheel, right_wheel);
    left_wheel = abs(left_wheel);
    right_wheel = abs(right_wheel);
    if (left_wheel > 255) {
        left_wheel = 255;
    }
    if (right_wheel > 255) {
        right_wheel = 255;
    }
    // 2 = diff_drive
    std::vector<int> send_msg = {2, left_wheel, left_dir, right_wheel, right_dir};
}

} // namespace
