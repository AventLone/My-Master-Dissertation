#pragma once
#include "api/MPC.h"
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
// #include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
// #include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


class LocalPlanner : public rclcpp::Node
{
public:
    explicit LocalPlanner(const std::string& name);
    ~LocalPlanner() = default;

private:
    MPC mMPC;   // Model prediction controller

    /* Parameters */
    static const int N{10};
    int z{0};
    Eigen::Vector3d mTargetState{-1.0, 4.0, M_PI / 2.0};
    Eigen::Vector3d mTargetStateClose{0.0, 0.0, 0.0};

    /* Flags */
    bool ref_path_close_set{false};
    bool mHavePlan{false}, mIsClose{false}, mIsGet{false}, mIsGrasp{false}, mIsAllTaskDown{false},
        mRobotStateSet{false}, mRefPathSet{false}, mIsEnd{false};

    /* Buffers */
    Eigen::Vector<double, 5> mCurrState;   // Current state
    Eigen::Matrix<double, N, 4> mGoalState;
    std::pair<Eigen::Matrix<double, 300, 4>, int> mDesiredGlobalPath;
    std::vector<Eigen::Vector2d> mObstacle;
    geometry_msgs::msg::Twist mControlCmd;
    visualization_msgs::msg::MarkerArray mObstacleMarkArray;

    // tf2::BufferCore tfBuffer;
    // tf2_ros::TransformListener tfListener{tfBuffer};

    /* Subscriptions */
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr mCurrStateSub, mObstacleSub, mGoalPathSub;

    /* Publishers */
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mLocalPathPub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mLocalPlanPub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mObstaclePub;

    rclcpp::TimerBase::SharedPtr mTimer;

    void drawObstacle();

    void chooseGoalState();

    void pubLocalPlan(const Eigen::MatrixXd& input_solution, const Eigen::MatrixXd& state_solution);

    void currStateCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg);

    void obstacleCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg);

    void globalPathCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg);

    void replanCallback();
};
