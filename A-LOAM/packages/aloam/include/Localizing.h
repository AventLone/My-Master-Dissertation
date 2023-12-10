#pragma once
#include "Mapping.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

#define DISTORTION 0

class Localizing
{
public:
    explicit Localizing();
    Localizing(const Localizing&) = delete;
    Localizing& operator=(const Localizing&) = delete;
    ~Localizing()
    {
        shutdown();
        if (mThread.joinable())
        {
            mThread.join();
        }
    }

    void insertCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& full_cloud,
                     const pcl::PointCloud<pcl::PointXYZI>::Ptr& sharp_cloud,
                     const pcl::PointCloud<pcl::PointXYZI>::Ptr& less_sharp_cloud,
                     const pcl::PointCloud<pcl::PointXYZI>::Ptr& flat_cloud,
                     const pcl::PointCloud<pcl::PointXYZI>::Ptr& less_flat_cloud);

    void setMapper(std::shared_ptr<Mapping> mapper)
    {
        mMapper = mapper;
    }

    void shutdown()
    {
        mShutDown = true;
        mLocalizeCondition.notify_all();
    }


private:
    int mCornerCorrespondence{0}, mPlaneCorrespondence{0};

    const double mScanPeriod{0.1};
    const double mDistanceSQthreshold{25};
    const double mNearbyScan{2.5};

    std::atomic<bool> mSystemInited{false};

    /*** Buffer ***/
    std::queue<pcl::PointCloud<pcl::PointXYZI>::Ptr> mFullCloudBuffer, mSharpCornerCloudBuffer,
        mLessSharpCornerCloudBuffer, mFlatSurfCloudBuffer, mLessFlatSurfCloudBuffer;

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr mCornerKDTree{new pcl::KdTreeFLANN<pcl::PointXYZI>()};
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr mSurfKDTree{new pcl::KdTreeFLANN<pcl::PointXYZI>()};

    pcl::PointCloud<PointType>::Ptr mCornerCloud{new pcl::PointCloud<PointType>()};
    pcl::PointCloud<PointType>::Ptr mSurfCloud{new pcl::PointCloud<PointType>()};

    int mCornerCloudNum{0}, mSurfCloudNum{0};

    // Transformation from current frame to world frame
    Eigen::Quaterniond q_w_curr{1, 0, 0, 0};
    Eigen::Vector3d t_w_curr{0, 0, 0};

    // q_curr_last(x, y, z, w), t_curr_last
    double para_q[4] = {0, 0, 0, 1};
    double para_t[3] = {0, 0, 0};

    Eigen::Map<Eigen::Quaterniond> q_last_curr{para_q};
    Eigen::Map<Eigen::Vector3d> t_last_curr{para_t};

    /*** Multi-Threaded part ***/
    std::mutex mBufferMutex;
    std::atomic<bool> mShutDown{false};
    std::condition_variable mLocalizeCondition;
    std::thread mThread;
    // boost::asio::thread_pool mThreadPool{3};

    std::shared_ptr<Mapping> mMapper{nullptr};

    // transform all lidar points to the start of the next frame
    void transformToStart(PointType const* const pi, PointType* const po) const;

    void transformToEnd(PointType const* const pi, PointType* const po) const
    {
        // undistort point first
        pcl::PointXYZI un_point_tmp;
        transformToStart(pi, &un_point_tmp);

        Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
        Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

        po->x = point_end.x();
        po->y = point_end.y();
        po->z = point_end.z();

        // Remove distortion time info
        po->intensity = pi->intensity;
    }

    void process();
};