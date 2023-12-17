#pragma once
#include "KeyFrame.h"
#include "SemanticSegmenting.h"
#include <atomic>
#include <condition_variable>
#include <thread>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace ORB_SLAM3
{
class PointCloudMapping
{
    using PointT = pcl::PointXYZRGB;
    using PointCloud = pcl::PointCloud<PointT>;

public:
    // PointCloudMapping(const std::string& setting_file);
    explicit PointCloudMapping(const std::string& setting_file, const std::string& seg_net_file);
    PointCloudMapping(const PointCloudMapping& T) = delete;
    PointCloudMapping& operator=(const PointCloudMapping& T) = delete;
    ~PointCloudMapping()
    {
        shutdown();
    }

    void insertKeyFrame(KeyFrame* kf);

    void updatePointCloud(Map& cur_map);

    void shutdown()
    {
        mShutdown = true;
        mKeyFrameUpdate.notify_all();
        if (mMyThread.joinable())
        {
            mMyThread.join();
        }
    }

    std::atomic<bool> mIsUpdating{false};   // 关于更新时的变量

    PointCloud::Ptr mGlobalMap{new PointCloud};

    std::vector<KeyFrame*> mCurrentKFs;

private:
    /* Buffers */
    std::queue<KeyFrame*> mKeyFrameBuffer;

    /*** Multi-Threaded ***/
    std::thread mMyThread;
    std::atomic<bool> mShutdown{false};
    std::condition_variable mKeyFrameUpdate;
    std::mutex mKeyframeMutex, mUpdateMutex, mGlobalMapMutex;

    // double resolution{0.05};
    // double meank{50};
    // double thresh{1};

    /* Point cloud filter */
    pcl::VoxelGrid<PointT> mVoxelFilter;
    pcl::StatisticalOutlierRemoval<PointT> mStatisticalFilter;

    // std::unique_ptr<TensorRT::SemanticSeger> mSegmenter{nullptr};


private:
    // void (PointCloudMapping::*processPointCloud)(KeyFrame* kf, PointCloud& cloud) const;

    // void processPointCloudWithSeg(KeyFrame* kf, PointCloud& cloud);
    // void processPointCloudWithoutSeg(KeyFrame* kf, PointCloud& cloud);

    void generatePointCloud(KeyFrame* kf);

    void run();
};
}   // namespace ORB_SLAM3
