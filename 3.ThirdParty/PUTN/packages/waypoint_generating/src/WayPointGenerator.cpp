#include "WayPointGenerator.h"
#include "sample_waypoints.hpp"
#include <rcpputils/asserts.hpp>
#include <tf2/utils.h>

WayPointGenerator::WayPointGenerator(const std::string& name) : rclcpp::Node(name)
{
    declare_parameter("waypoint_type", "manual-lonely-waypoint");
    get_parameter("waypoint_type", waypoint_type);

    mOdomSub = create_subscription<nav_msgs::msg::Odometry>(
        "orb_slam3/odom",
        rclcpp::ServicesQoS().reliable(),
        std::bind(&WayPointGenerator::odomCallback, this, std::placeholders::_1));
    mGoalSub = create_subscription<geometry_msgs::msg::PoseStamped>(
        // "goal",
        "goal_pose",
        rclcpp::ServicesQoS().reliable(),
        std::bind(&WayPointGenerator::goalCallback, this, std::placeholders::_1));
    mTrajStartTriggerSub = create_subscription<geometry_msgs::msg::PoseStamped>(
        "traj_start_trigger",
        rclcpp::ServicesQoS().reliable(),
        std::bind(&WayPointGenerator::trajStartTriggerCallback, this, std::placeholders::_1));

    mWayPointsPub = create_publisher<nav_msgs::msg::Path>("putn/waypoints", rclcpp::ServicesQoS().reliable());
    mWayPointsVisPub =
        create_publisher<geometry_msgs::msg::PoseArray>("putn/waypoints_vis", rclcpp::ServicesQoS().best_effort());
}

void WayPointGenerator::loadSeg(int seg_id, const rclcpp::Time& time_base)
{
    std::string seg_str = boost::str(bfmt("seg%d/") % seg_id);
    double yaw;
    double time_from_start;
    RCLCPP_INFO(get_logger(), "Getting segment %d", seg_id);
    rcpputils::assert_true(get_parameter(seg_str + "yaw", yaw));
    // RCLCPP_ASSERT_MSG((yaw > -3.1499999) && (yaw < 3.14999999), "yaw=%.3f", yaw);
    rcpputils::assert_true((yaw > -3.1499999) && (yaw < 3.14999999), "yaw is beyond the range");
    rcpputils::assert_true(get_parameter(seg_str + "time_from_start", time_from_start));
    rcpputils::assert_true(time_from_start >= 0.0);

    std::vector<double> ptx;
    std::vector<double> pty;
    std::vector<double> ptz;

    rcpputils::assert_true(get_parameter(seg_str + "x", ptx));
    get_parameter(seg_str + "y", pty);
    get_parameter(seg_str + "z", ptz);

    rcpputils::assert_true(ptx.size());
    rcpputils::assert_true(ptx.size() == pty.size() && ptx.size() == ptz.size());

    nav_msgs::msg::Path path_msg;

    path_msg.header.stamp = time_base + rclcpp::Duration::from_seconds(time_from_start);

    double baseyaw = tf2::getYaw(mOdom.pose.pose.orientation);

    for (size_t k = 0; k < ptx.size(); ++k)
    {
        geometry_msgs::msg::PoseStamped pt;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, baseyaw + yaw);
        // pt.pose.orientation = tf2::createQuaternionMsgFromYaw(baseyaw + yaw);
        pt.pose.orientation = tf2::toMsg(q);
        Eigen::Vector2d dp(ptx.at(k), pty.at(k));
        Eigen::Vector2d rdp;
        rdp.x() = std::cos(-baseyaw - yaw) * dp.x() + std::sin(-baseyaw - yaw) * dp.y();
        rdp.y() = -std::sin(-baseyaw - yaw) * dp.x() + std::cos(-baseyaw - yaw) * dp.y();
        pt.pose.position.x = rdp.x() + mOdom.pose.pose.position.x;
        pt.pose.position.y = rdp.y() + mOdom.pose.pose.position.y;
        pt.pose.position.z = ptz.at(k) + mOdom.pose.pose.position.z;
        path_msg.poses.push_back(pt);
    }

    waypointSegments.push_back(path_msg);
}

void WayPointGenerator::loadWayPoints(const rclcpp::Time& time_base)
{
    int seg_cnt = 0;
    waypointSegments.clear();
    rcpputils::assert_true(get_parameter("segment_cnt", seg_cnt));
    for (int i = 0; i < seg_cnt; ++i)
    {
        loadSeg(i, time_base);
        if (i > 0)
        {
            rcpputils::assert_true(static_cast<rclcpp::Time>(waypointSegments[i - 1].header.stamp) <
                                   static_cast<rclcpp::Time>(waypointSegments[i].header.stamp));
        }
    }
    RCLCPP_INFO(get_logger(), "Overall load %zu segments", waypointSegments.size());
}

void WayPointGenerator::pubWayPoints()
{
    mWayPoints.header.frame_id = "world";
    mWayPoints.header.stamp = this->now();
    mWayPointsPub->publish(mWayPoints);
    geometry_msgs::msg::PoseStamped init_pose;
    init_pose.header = mOdom.header;
    init_pose.pose = mOdom.pose.pose;
    mWayPoints.poses.insert(mWayPoints.poses.begin(), init_pose);
    mWayPointsPub->publish(mWayPoints);
    mWayPoints.poses.clear();
}

void WayPointGenerator::pubWayPointsVis()
{
    nav_msgs::msg::Path wp_vis = mWayPoints;
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = "world";
    pose_array.header.stamp = this->now();
    {
        geometry_msgs::msg::Pose init_pose;
        init_pose = mOdom.pose.pose;
        pose_array.poses.push_back(init_pose);
    }
    for (auto it = mWayPoints.poses.begin(); it != mWayPoints.poses.end(); ++it)
    {
        geometry_msgs::msg::Pose p;
        p = it->pose;
        pose_array.poses.push_back(p);
    }
    mWayPointsVisPub->publish(pose_array);
}

void WayPointGenerator::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    mIsOdomReady = true;
    mOdom = *msg;

    if (waypointSegments.size())
    {
        rclcpp::Time expected_time = waypointSegments.front().header.stamp;

        if (static_cast<rclcpp::Time>(mOdom.header.stamp).seconds() >= expected_time.seconds())
        {
            mWayPoints = waypointSegments.front();

            std::stringstream ss;
            ss << bfmt("Series send %.3f from start:\n") % mTriggedTime.seconds();
            for (auto& pose_stamped : mWayPoints.poses)
            {
                ss << bfmt("P[%.2f, %.2f, %.2f] q(%.2f,%.2f,%.2f,%.2f)") % pose_stamped.pose.position.x %
                          pose_stamped.pose.position.y % pose_stamped.pose.position.z %
                          pose_stamped.pose.orientation.w % pose_stamped.pose.orientation.x %
                          pose_stamped.pose.orientation.y % pose_stamped.pose.orientation.z
                   << std::endl;
            }
            RCLCPP_INFO_STREAM(get_logger(), ss.str());
            pubWayPointsVis();
            pubWayPoints();
            waypointSegments.pop_front();
        }
    }
}

void WayPointGenerator::goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
{
    mTriggedTime = this->now();   // odom.header.stamp;
    // rcpputils::assert_true(mTriggedTime > rclcpp::Time(0, 0));

    // n.param("waypoint_type", waypoint_type, "manual");
    get_parameter("waypoint_type", waypoint_type);

    if (waypoint_type == "circle")
    {
        mWayPoints = circle();
        pubWayPointsVis();
        pubWayPoints();
    }
    else if (waypoint_type == "eight")
    {
        mWayPoints = eight();
        pubWayPointsVis();
        pubWayPoints();
    }
    else if (waypoint_type == "point")
    {
        mWayPoints = point();
        pubWayPointsVis();
        pubWayPoints();
    }
    else if (waypoint_type == "series")
    {
        loadWayPoints(mTriggedTime);
    }
    else if (waypoint_type == "manual-lonely-waypoint")
    {
        std::cout << "recieve_x:  " << msg->pose.position.x << std::endl;
        std::cout << "recieve_y:  " << msg->pose.position.y << std::endl;
        std::cout << "recieve_z:  " << msg->pose.position.z << std::endl;
        if (msg->pose.position.z >= 0)
        {
            // if height >= 0, it's a valid goal;
            geometry_msgs::msg::PoseStamped pt = *msg;
            mWayPoints.poses.clear();
            mWayPoints.poses.push_back(pt);
            pubWayPointsVis();
            pubWayPoints();
        }
        else
        {
            RCLCPP_WARN(get_logger(), "invalid goal in manual-lonely-waypoint mode.");
        }
    }
    else
    {
        if (msg->pose.position.z > 0)
        {
            // if height > 0, it's a normal goal;
            geometry_msgs::msg::PoseStamped pt = *msg;
            if (waypoint_type == "noyaw")
            {
                // double yaw = tf2::impl::getYaw(mOdom.pose.pose.orientation);
                double yaw = tf2::getYaw(mOdom.pose.pose.orientation);
                tf2::Quaternion quat;
                quat.setRPY(0, 0, yaw);
                tf2::convert(quat, pt.pose.orientation);
            }
            mWayPoints.poses.push_back(pt);
            pubWayPointsVis();
        }
        else if (msg->pose.position.z > -1.0)
        {
            // If 0 > height > -1.0, remove last goal;
            if (mWayPoints.poses.size() >= 1)
            {
                mWayPoints.poses.erase(std::prev(mWayPoints.poses.end()));
            }
            pubWayPointsVis();
        }
        else
        {
            // If -1.0 > height, end of input.
            if (mWayPoints.poses.size() >= 1)
            {
                pubWayPoints();
                pubWayPointsVis();
            }
        }
    }
}

void WayPointGenerator::trajStartTriggerCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
{
    if (!mIsOdomReady)
    {
        RCLCPP_ERROR(get_logger(), "No odom!");
        return;
    }

    RCLCPP_WARN(get_logger(), "Trigger!");
    mTriggedTime = mOdom.header.stamp;

    // rcpputils::assert_true(mTriggedTime > rclcpp::Time(0, 0));
    rcpputils::assert_true(mTriggedTime.seconds() > 0.0);

    // ros::NodeHandle n("~");
    // n.param("waypoint_type", waypoint_type, string("manual"));
    get_parameter("waypoint_type", waypoint_type);

    RCLCPP_ERROR_STREAM(get_logger(), "Pattern " << waypoint_type << " generated!");
    if (waypoint_type == "free")
    {
        mWayPoints = point();
        pubWayPointsVis();
        pubWayPoints();
    }
    else if (waypoint_type == "circle")
    {
        mWayPoints = circle();
        pubWayPointsVis();
        pubWayPoints();
    }
    else if (waypoint_type == "eight")
    {
        mWayPoints = eight();
        pubWayPointsVis();
        pubWayPoints();
    }
    else if (waypoint_type == "point")
    {
        mWayPoints = point();
        pubWayPointsVis();
        pubWayPoints();
    }
    else if (waypoint_type == "series")
    {
        loadWayPoints(mTriggedTime);
    }
}
