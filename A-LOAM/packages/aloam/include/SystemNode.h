#pragma once
#include "Localizing.h"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

class SystemNode : public rclcpp::Node
{
public:
    explicit SystemNode(const std::string& name);
    SystemNode(const SystemNode&) = delete;
    SystemNode& operator=(const SystemNode&) = delete;
    ~SystemNode() = default;

private:
    const double mScanPeriod{0.1};

    const int mSystemDelay{0};
    int mSystemInitCount{0};
    bool mSystemInited{false};

    float mCloudCurvature[400000];
    int cloudSortInd[400000];
    int mCloudNeighborPicked[400000];
    int mCloudLabel[400000];

    int mScanLines{16};
    double mMinimumRange{0.1};

    std::shared_ptr<Localizing> mLocalizer{nullptr};
    std::shared_ptr<Mapping> mMapper{nullptr};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mRawCloudSub;

    void cloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};