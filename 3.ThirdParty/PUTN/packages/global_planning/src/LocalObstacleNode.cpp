#include "LocalObstacleNode.h"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>   //if don't include this header, it throw exceptions while compiling

LocalObstacleNode::LocalObstacleNode(const std::string& name) : rclcpp::Node(name)
{
    declare_parameter("LocalMap.Resolution", 0.1);
    declare_parameter("LocalMap.local_x_l", -2.0);
    declare_parameter("LocalMap.local_x_u", 2.0);
    declare_parameter("LocalMap.local_y_l", -2.0);
    declare_parameter("LocalMap.local_y_u", 2.0);
    declare_parameter("LocalMap.local_z_l", -0.3);
    declare_parameter("LocalMap.local_z_u", 0.5);

    get_parameter("LocalMap.Resolution", mResolution);
    get_parameter("LocalMap.local_x_l", mLocal_x_l);
    get_parameter("LocalMap.local_x_u", mLocal_x_u);
    get_parameter("LocalMap.local_y_l", mLocal_y_l);
    get_parameter("LocalMap.local_y_u", mLocal_y_u);
    get_parameter("LocalMap.local_z_l", mLocal_z_l);
    get_parameter("LocalMap.local_z_u", mLocal_z_u);


    mTfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);

    mScanSub = create_subscription<sensor_msgs::msg::PointCloud2>(
        "BD_Roamer/depth_camera/point_cloud",
        rclcpp::SensorDataQoS().best_effort(),
        std::bind(&LocalObstacleNode::scanCallback, this, std::placeholders::_1));

    mObstacleVisPub = create_publisher<sensor_msgs::msg::PointCloud2>("putn/obstacle_vis",
                                                                      rclcpp::ParameterEventsQoS().best_effort());
    mObstaclePub =
        create_publisher<std_msgs::msg::Float32MultiArray>("putn/obstacle", rclcpp::ParameterEventsQoS().reliable());
}

void LocalObstacleNode::scanCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    // RCLCPP_INFO(get_logger(), "Receive Velodyne Lidar scan!");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> passthrough;
    passthrough.setInputCloud(cloud);
    passthrough.setFilterFieldName("x");
    passthrough.setFilterLimits(mLocal_x_l, mLocal_x_u);
    passthrough.filter(*cloud_after_PassThrough);

    passthrough.setInputCloud(cloud_after_PassThrough);
    passthrough.setFilterFieldName("y");
    passthrough.setFilterLimits(mLocal_y_l, mLocal_y_u);
    passthrough.filter(*cloud_after_PassThrough);

    passthrough.setInputCloud(cloud_after_PassThrough);
    passthrough.setFilterFieldName("z");
    passthrough.setFilterLimits(mLocal_z_l, mLocal_z_u);
    passthrough.filter(*cloud_after_PassThrough);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filt(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Vector3d lowerbound(mLocal_x_l, mLocal_y_l, mLocal_z_l);
    Eigen::Vector3d upperbound(mLocal_x_u, mLocal_y_u, mLocal_z_u);
    putn::World local_world = putn::World(mResolution);
    local_world.initGridMap(lowerbound, upperbound);
    for (const auto& pt : (*cloud_after_PassThrough).points)
    {
        Eigen::Vector3d obstacle(pt.x, pt.y, pt.z);
        if (local_world.isFree(obstacle))
        {
            local_world.setObstacle(obstacle);

            Eigen::Vector3d obstacle_round = local_world.coordRounding(obstacle);
            pcl::PointXYZ pt_add;
            pt_add.x = obstacle_round(0);
            pt_add.y = obstacle_round(1);
            pt_add.z = obstacle_round(2);
            cloud_filt->points.push_back(pt_add);
        }
    }

    // mTfListener->waitForTransform("/world", "/aft_mapped", ros::Time(0), ros::Duration(2.0));
    // std::function<void(const tf2_ros::TransformStampedFuture&)> f = [&]() {};
    // mTfBuffer->waitForTransform("world", "odom", tf2::TimePointZero, std::chrono::seconds(2), f);

    //-----------------
    // Wait for the transformation from "/world" to "/odom"
    try
    {
        mTfBuffer->canTransform("world", "aft_mapped", tf2::TimePointZero, tf2::durationFromSec(2.0));
    }
    catch (tf2::TransformException& e)
    {
        RCLCPP_ERROR(get_logger(), "Transform exception: %s", e.what());
        return;
    }
    //-----------------


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tran(new pcl::PointCloud<pcl::PointXYZ>);

    std_msgs::msg::Float32MultiArray obs_array;
    for (const auto& pt : cloud_filt->points)
    {
        geometry_msgs::msg::PointStamped origin_point;
        origin_point.header.frame_id = "odom";
        origin_point.header.stamp = this->now();
        origin_point.point.x = pt.x;
        origin_point.point.y = pt.y;
        origin_point.point.z = pt.z;

        geometry_msgs::msg::PointStamped trans_point;

        // mTfListener.transformPoint("world", origin_point, trans_point);

        /* Transform the point from "/world" to "/odom" */
        try
        {
            mTfBuffer->transform(origin_point, trans_point, "world", tf2::durationFromSec(2.0));
            // trans_point = mTfBuffer->transform(origin_point, "world", tf2::durationFromSec(2.0));
            // trans_point = mTfBuffer->transform(origin_point, "world", std::chrono::seconds(2));
        }
        catch (const tf2::TransformException& e)
        {
            RCLCPP_ERROR(get_logger(), "Transform exception: %s", e.what());
            return;
        }

        pcl::PointXYZ point;
        if (!(-1.2 < pt.x && pt.x < 0.4 && -0.4 < pt.y && pt.y < 0.4))
        {
            obs_array.data.push_back(trans_point.point.x);
            obs_array.data.push_back(trans_point.point.y);
            obs_array.data.push_back(trans_point.point.z);
        }

        point.x = trans_point.point.x;
        point.y = trans_point.point.y;
        point.z = trans_point.point.z;

        cloud_tran->points.push_back(point);
    }

    sensor_msgs::msg::PointCloud2 obs_vis;
    pcl::toROSMsg(*cloud_tran, obs_vis);

    obs_vis.header.frame_id = "world";
    mObstacleVisPub->publish(obs_vis);
    mObstaclePub->publish(obs_array);
}
