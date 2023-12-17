#pragma once
#include <webots_ros2_driver/WebotsNode.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/inertia_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>


class RobotDriver : public webots_ros2_driver::PluginInterface
{
public:
    void step() override;
    void init(webots_ros2_driver::WebotsNode* node, std::unordered_map<std::string, std::string>& parameters) override;

private:
    /* Parameters */
    const double mHalfDistanceBetweenWheels{0.045}, mWheelRadius{1.0};
    const int mTimeStep{1};

    /* The robot in Webots */
    std::unique_ptr<webots::Robot> mRobot{nullptr};

    /* The actuators on the robot. */
    webots::Motor* mLeftMotor{nullptr};
    webots::Motor* mRightMotor{nullptr};

    /* The sensors on the robot. */
    webots::Gyro* mGyro{nullptr};
    webots::Accelerometer* mAccelerometer{nullptr};

    /* Buffers */
    sensor_msgs::msg::Imu mImuMsg;
    geometry_msgs::msg::Twist mCmdVelMsg;

    /* rclcpp::Node, Subscriptions and Publishers. */
    webots_ros2_driver::WebotsNode* mNode;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mCmdVelSub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr mImuPub;
    rclcpp::TimerBase::SharedPtr mTimer;

    void publishImu();
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
};
