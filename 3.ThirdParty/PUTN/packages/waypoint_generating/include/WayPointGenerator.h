#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>

#include <vector>
#include <deque>
#include <boost/format.hpp>
#include <Eigen/Dense>

// using namespace std;
using bfmt = boost::format;

class WayPointGenerator : public rclcpp::Node
{
    // waypoint type
    enum WayPointType : char
    {
        FREE,
        CIRCLE,
        EIGHT,
        POINT,
        SERIES,
        MANUAL,
    };

public:
    WayPointGenerator(const std::string& name);

    std::unordered_map<std::string, WayPointType> mWayPointTypeDict{
        {"free", FREE}, {"circle", CIRCLE}, {"eight", EIGHT}, {"point", POINT}, {"series", SERIES}, {"manual", MANUAL}};

private:
    bool mIsOdomReady;
    rclcpp::Time mTriggedTime{0, 0};

    std::string waypoint_type = "manual";

    std::deque<nav_msgs::msg::Path> waypointSegments;

    nav_msgs::msg::Path mWayPoints;
    nav_msgs::msg::Odometry mOdom;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mOdomSub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mGoalSub, mTrajStartTriggerSub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mWayPointsPub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr mWayPointsVisPub;

    void loadSeg(int seg_id, const rclcpp::Time& time_base);

    void loadWayPoints(const rclcpp::Time& time_base);

    void pubWayPoints();

    void pubWayPointsVis();

    /* RCLCPP Callback functions */
    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
    void trajStartTriggerCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
};