#pragma once
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/imu.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <pcl_conversions/pcl_conversions.h>

#include "api/include/System.h"
#include "OccupancyMapping.h"


class SlamNode : public rclcpp::Node
{
public:
    explicit SlamNode(const std::string& name);

    ~SlamNode();

    void setOccupancyMapper(const std::shared_ptr<OccupancyMapping>& occupancy_mapper)
    {
        mOccupancyMapper = occupancy_mapper;
    }

private:
    /*** Parameters ***/
    Eigen::Affine3f mTcr;   // Coefficient to transform pointcloud onto rviz2 frame
    Eigen::Matrix4f mTrviz, mTodom;

    /*** SLAM ***/
    std::unique_ptr<ORB_SLAM3::System> mSlamer;
    Sophus::SE3f mTcw;   // The transform matrix of the world fram to the camera frame
    std::vector<ORB_SLAM3::IMU::Point> mImuMeas;
    std::shared_ptr<OccupancyMapping> mOccupancyMapper;

    /*** Map and Odom **/
    pcl::PointCloud<pcl::PointXYZRGB> mPointCloudMap;
    std::shared_ptr<octomap::ColorOcTree> mOctoMap;

    /************************************* RCLCPP *************************************/
    /*** Synchronized Subsribers ***/
    using SyncPolicy =
        message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    message_filters::Subscriber<sensor_msgs::msg::Image>* mRgbSub;
    message_filters::Subscriber<sensor_msgs::msg::Image>* mDepthSub;
    message_filters::Synchronizer<SyncPolicy>* mSynchronizer;
    /*** Subsribers ***/
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr mImuSub;
    /*** Publishers ***/
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr mOctomapPublisher;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPointCloudPublisher;
    /*** Transform Broadcaster ***/
    std::unique_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;
    /*** Timer ***/
    rclcpp::TimerBase::SharedPtr mTimer[2];
    /*** Messages ***/
    // sensor_msgs::msg::PointCloud2 mPointCloudMsg;
    geometry_msgs::msg::TransformStamped mTfMsg;
    octomap_msgs::msg::Octomap mOctomapMsg;
    /*** Callback policy, Multi-Threaded or Single-Threaded ***/
    rclcpp::CallbackGroup::SharedPtr mMutiThreadCallback, mSingleThreadCallback;
    /**********************************************************************************/

    /*** Multi-Threaded ***/

private:
    /*** Subscription callbacks ***/
    void rgbDepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                          const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);
    void subImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg);

    /*** Timer callbacks ***/
    void publishOctoMap();
    void publishOdom();
};