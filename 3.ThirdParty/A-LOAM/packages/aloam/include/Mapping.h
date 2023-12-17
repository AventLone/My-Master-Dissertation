#pragma once
#include "common.hpp"
#include "LidarFactor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <queue>
#include <mutex>
#include <boost/asio.hpp>   // Thread pool

class Mapping : public rclcpp::Node
{
public:
    explicit Mapping(float line_resolution, float plane_resolution);
    Mapping(const Mapping&) = delete;
    Mapping& operator=(const Mapping&) = delete;
    ~Mapping()
    {
        shutdown();
        if (mThread.joinable())
        {
            mThread.join();
        }
        mThreadPool.join();
    }

    void insertCloudAndOdom(const pcl::PointCloud<pcl::PointXYZI>::Ptr& full_cloud,
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr& corner_cloud,
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_cloud,
                            const Eigen::Quaterniond& orientation, const Eigen::Vector3d& position);

    void shutdown()
    {
        mShutDown = true;
        mCondition.notify_all();
    }

private:
    // int mFrameCount{0};

    int mCloudCenWidth{10};
    int mCloudCenHeight{10};
    int mCloudCenDepth{5};
    const int mCloudWidth{21};
    const int mCloudHeight{21};
    const int mCloudDepth{11};

    const int mCloudNum = mCloudWidth * mCloudHeight * mCloudDepth;   // 4851

    int mCloudValidInd[125];
    int mCloudSurroundInd[125];

    // input: from odom
    // pcl::PointCloud<PointType>::Ptr corner_cloud{new pcl::PointCloud<PointType>()};
    // pcl::PointCloud<PointType>::Ptr surf_cloud{new pcl::PointCloud<PointType>()};

    /* Ouput: all visualble cube points */
    pcl::PointCloud<PointType>::Ptr mSurroundCloud{new pcl::PointCloud<PointType>()};

    /* Surround points in map to build tree. */
    pcl::PointCloud<PointType>::Ptr mCornerFromMapCloud{new pcl::PointCloud<PointType>()};
    pcl::PointCloud<PointType>::Ptr mSurfFromMapCloud{new pcl::PointCloud<PointType>()};

    // input & output: points in one frame. local --> global
    // pcl::PointCloud<PointType>::Ptr full_cloud{new pcl::PointCloud<PointType>()};

    std::vector<pcl::PointCloud<PointType>::Ptr> mCornerCloudArray, mSurfCloudArray;

    // kd-tree
    pcl::KdTreeFLANN<PointType>::Ptr mCornerFromMapKDTree{new pcl::KdTreeFLANN<PointType>()};
    pcl::KdTreeFLANN<PointType>::Ptr mSurfFromMapKDTree{new pcl::KdTreeFLANN<PointType>()};

    double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
    Eigen::Map<Eigen::Quaterniond> q_w_curr{parameters};
    Eigen::Map<Eigen::Vector3d> t_w_curr{parameters + 4};

    // transformation between odom's world and map's world frame
    Eigen::Quaterniond q_wmap_wodom{1, 0, 0, 0};
    Eigen::Vector3d t_wmap_wodom{0, 0, 0};

    Eigen::Quaterniond q_wodom_curr{1, 0, 0, 0};
    Eigen::Vector3d t_wodom_curr{0, 0, 0};

    pcl::VoxelGrid<PointType> mCornerDownSizeFilter, mSurfDownSizeFilter;

    std::vector<int> mPointSearchInd;
    std::vector<float> mPointSearchSqDis;

    pcl::PointXYZI pointOri, pointSel;

    nav_msgs::msg::Path mPath;

    /*** Buffer ***/
    std::queue<pcl::PointCloud<pcl::PointXYZI>::Ptr> mFullCloudBuffer, mCornerCloudBuffer, mSurfCloudBuffer;
    std::queue<Eigen::Quaterniond> mOrientationBuffer;
    std::queue<Eigen::Vector3d> mPositionBuffer;

    std::mutex mBufferMutex;
    std::atomic<bool> mShutDown{false};
    std::thread mThread;
    std::condition_variable mCondition;
    boost::asio::thread_pool mThreadPool{1};

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mCloudMapPub, mSurroundCloudPub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mOdomPub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mPathPub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;


    // set initial guess
    void transformAssociateToMap()
    {
        q_w_curr = q_wmap_wodom * q_wodom_curr;
        t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
    }

    void transformUpdate()
    {
        q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
        t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
    }

    void pointAssociateToMap(PointType const* const pi, PointType* const po)
    {
        Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
        po->x = point_w.x();
        po->y = point_w.y();
        po->z = point_w.z();
        po->intensity = pi->intensity;
    }

    void pointAssociateTobeMapped(PointType const* const pi, PointType* const po)
    {
        Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
        po->x = point_curr.x();
        po->y = point_curr.y();
        po->z = point_curr.z();
        po->intensity = pi->intensity;
    }

    void publishMsgs(Eigen::Quaterniond orientation, Eigen::Vector3d position,
                     pcl::PointCloud<PointType>::Ptr surround_cloud, pcl::PointCloud<PointType>::Ptr cloud_map);

    void run();
};