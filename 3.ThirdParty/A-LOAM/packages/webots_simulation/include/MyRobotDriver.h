#pragma once
#include <rclcpp/macros.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
// #include <webots/InertialUnit.hpp>
// #include <webots/Gyro.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/inertia_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>


class MyRobotDriver : public webots_ros2_driver::PluginInterface
{
public:
    void step() override;
    void init(webots_ros2_driver::WebotsNode* node, std::unordered_map<std::string, std::string>& parameters) override;

private:
    const double mHalfDistanceBetweenWheels{0.045}, mWheelRadius{1.0};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mCmdVelSubscriber;
    geometry_msgs::msg::Twist mCmdVelMsg;

    std::unique_ptr<webots::Robot> mRobot;

    const int mTimeStep{1};
    webots::Motor* mLeftMotor{nullptr};
    webots::Motor* mRightMotor{nullptr};

    webots::Gyro* mGyro{nullptr};
    webots::Accelerometer* mAccelerometer{nullptr};

    sensor_msgs::msg::Imu mImuMsg;

    webots_ros2_driver::WebotsNode* mNode;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr mImuPublisher;
    rclcpp::TimerBase::SharedPtr mTimer;


    void publishImu();
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
};
