#include "MyRobotDriver.h"
#include <webots/Gyro.hpp>
#include <webots/Accelerometer.hpp>


void MyRobotDriver::init(webots_ros2_driver::WebotsNode* node, std::unordered_map<std::string, std::string>& parameters)
{
    mNode = node;
    mRobot = std::make_unique<webots::Robot>();

    mLeftMotor = mRobot->getMotor("left track motor");
    mRightMotor = mRobot->getMotor("right track motor");

    /*** Imu ***/
    mGyro = mRobot->getGyro("gyro");
    mGyro->enable(mTimeStep);
    mAccelerometer = mRobot->getAccelerometer("accelerometer");
    mAccelerometer->enable(mTimeStep);

    mLeftMotor->setPosition(INFINITY);
    mRightMotor->setPosition(INFINITY);

    mLeftMotor->setVelocity(0.0);
    mRightMotor->setVelocity(0.0);

    mCmdVelSubscriber = mNode->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        rclcpp::SensorDataQoS().reliable(),
        std::bind(&MyRobotDriver::cmdVelCallback, this, std::placeholders::_1));

    mImuPublisher =
        mNode->create_publisher<sensor_msgs::msg::Imu>("track_cart/imu", rclcpp::SensorDataQoS().reliable());
    mTimer = mNode->create_wall_timer(std::chrono::milliseconds(25), std::bind(&MyRobotDriver::publishImu, this));
}

void MyRobotDriver::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    mCmdVelMsg.linear = msg->linear;
    mCmdVelMsg.angular = msg->angular;
}

void MyRobotDriver::publishImu()
{
    /*** Publish imu data ***/
    auto angular_vel = mGyro->getValues();
    auto accele = mAccelerometer->getValues();
    mImuMsg.angular_velocity.x = angular_vel[0];
    mImuMsg.angular_velocity.y = angular_vel[1];
    mImuMsg.angular_velocity.z = angular_vel[2];
    mImuMsg.linear_acceleration.x = accele[0];
    mImuMsg.linear_acceleration.y = accele[1];
    mImuMsg.linear_acceleration.z = accele[2];
    mImuMsg.header.stamp = mNode->now();
    mImuPublisher->publish(mImuMsg);
}

void MyRobotDriver::step()
{
    auto linear_speed = mCmdVelMsg.linear.x;
    auto angular_speed = mCmdVelMsg.angular.z;

    auto command_motor_left = (linear_speed - angular_speed * mHalfDistanceBetweenWheels) / mWheelRadius;
    auto command_motor_right = (linear_speed + angular_speed * mHalfDistanceBetweenWheels) / mWheelRadius;

    mLeftMotor->setVelocity(command_motor_left);
    mRightMotor->setVelocity(command_motor_right);
}


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(MyRobotDriver, webots_ros2_driver::PluginInterface)