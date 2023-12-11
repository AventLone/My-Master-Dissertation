#include "LocalObstacleNode.h"
#include <geometry_msgs/msg/point_stamped.hpp>

LocalObstacleNode::LocalObstacleNode(const std::string& name) : rclcpp::Node(name)
{
    // mTfListener = std::make_unique<tf2_ros::TransformListener>(this);

    mMapSub = create_subscription<sensor_msgs::msg::PointCloud2>(
        "map",
        rclcpp::SensorDataQoS().reliable(),
        std::bind(&LocalObstacleNode::mapCallback, this, std::placeholders::_1));

    mObstacleVisPub =
        create_publisher<sensor_msgs::msg::PointCloud2>("obs_vis", rclcpp::ParameterEventsQoS().reliable());
    mObstaclePub = create_publisher<std_msgs::msg::Float32MultiArray>("obs", rclcpp::ParameterEventsQoS().reliable());
}

void LocalObstacleNode::mapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    RCLCPP_INFO(get_logger(), "Receive velodyne!");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> passthrough;
    passthrough.setInputCloud(cloud);
    passthrough.setFilterFieldName("x");
    passthrough.setFilterLimits(local_x_l, local_x_u);
    passthrough.filter(*cloud_after_PassThrough);

    passthrough.setInputCloud(cloud_after_PassThrough);
    passthrough.setFilterFieldName("y");
    passthrough.setFilterLimits(local_y_l, local_y_u);
    passthrough.filter(*cloud_after_PassThrough);

    passthrough.setInputCloud(cloud_after_PassThrough);
    passthrough.setFilterFieldName("z");
    passthrough.setFilterLimits(local_z_l, local_z_u);
    passthrough.filter(*cloud_after_PassThrough);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filt(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Vector3d lowerbound(local_x_l, local_y_l, local_z_l);
    Eigen::Vector3d upperbound(local_x_u, local_y_u, local_z_u);
    putn::World local_world = putn::World(resolution);
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

    geometry_msgs::msg::TransformStamped transformStamped;

    try
    {
        transformStamped = mTfBuffer.lookupTransform("world", "aft_mapped", tf2::TimePointZero);
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_INFO(this->get_logger(), "Could not transform world to aft_mapped: %s", ex.what());
        return;
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tran(new pcl::PointCloud<pcl::PointXYZ>);

    std_msgs::msg::Float32MultiArray obs_array;
    for (const auto& pt : cloud_filt->points)
    {
        geometry_msgs::msg::PointStamped origin_point;
        origin_point.header.frame_id = "aft_mapped";
        origin_point.header.stamp = this->now();
        origin_point.point.x = pt.x;
        origin_point.point.y = pt.y;
        origin_point.point.z = pt.z;

        geometry_msgs::msg::PointStamped trans_point;

        mTfListener.transformPoint("world", origin_point, trans_point);

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
