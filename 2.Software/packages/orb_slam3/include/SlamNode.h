#pragma once
#include "api/include/System.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_broadcaster.h>

class SlamNode : public rclcpp::Node
{
public:
    explicit SlamNode(const std::string& name);

    ~SlamNode()
    {
        delete mRgbSub;
        delete mDepthSub;
        delete mSynchronizer;
        mSlamer->Shutdown();
    }

private:
    /*** Parameters ***/
    Eigen::Affine3f mTcr;   // Coefficient to transform pointcloud onto rviz2 frame
    Eigen::Matrix4f mTrviz, mTodom;

    /*** SLAM ***/
    std::unique_ptr<ORB_SLAM3::System> mSlamer;

    /*** Buffers ***/
    nav_msgs::msg::Path mPathMsg;
    Sophus::SE3f mTcw;   // The transform matrix of the world fram to the camera frame
    std::vector<ORB_SLAM3::IMU::Point> mImuMeas;

    /*** Synchronized Subsribers ***/
    using SyncPolicy =
        message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    message_filters::Subscriber<sensor_msgs::msg::Image>* mRgbSub;
    message_filters::Subscriber<sensor_msgs::msg::Image>* mDepthSub;
    message_filters::Synchronizer<SyncPolicy>* mSynchronizer;

    /*** Subscriptions ***/
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr mImuSub;

    /*** Publishers ***/
    // rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr mOctomapPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mCloudMapPub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mOdomPub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mPathPub;

    /*** Transform Broadcaster ***/
    std::unique_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;

    /*** Timer ***/
    rclcpp::TimerBase::SharedPtr mTimer[2];

    /*** Callback policy, Multi-Threaded or Single-Threaded ***/
    rclcpp::CallbackGroup::SharedPtr mMutiThreadCallback, mSingleThreadCallback;

private:
    /*** Subscription callbacks ***/
    void rgbDepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                          const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);
    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg);

    /*** Timer callbacks ***/
    void publishNavMsgs();
    // void publishOctoMap();
    void publishCloudMap();
};