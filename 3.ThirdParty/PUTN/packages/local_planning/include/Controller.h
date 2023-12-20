#pragma once
#include <Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>

class Controller : public rclcpp::Node
{
public:
    explicit Controller(const std::string& name);
    ~Controller() = default;

private:
    /* Parameters */
    const int N{10};   // Prediction Horizon

    /* Buffers */
    Eigen::Vector4d mCurrState;
    Eigen::MatrixXd mLocalPlan;

    geometry_msgs::msg::Twist mControlCmdMsg;

    /* Subscriptions */
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr mLocalPlanSub;

    /* Publishers */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mVelCmdPub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mCurrStatePub;
    rclcpp::TimerBase::SharedPtr mTimer;

    void localPlanCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg);
};