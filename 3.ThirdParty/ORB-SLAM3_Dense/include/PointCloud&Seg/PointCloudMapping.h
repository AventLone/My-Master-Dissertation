#pragma once

#include <atomic>
#include <condition_variable>
#include <thread>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "KeyFrame.h"
#include "SemanticSegmenting.h"

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

    ~PointCloudMapping() = default;

    void insertKeyFrame(KeyFrame* kf);

    void updatePointCloud(Map& cur_map);

    void shutdown();

public:
    // 关于更新时的变量
    std::atomic<bool> mIsUpdating{false};

    PointCloud mGlobalMap;

    std::vector<KeyFrame*> mCurrentKFs;

private:
    /*** Multi-Threaded ***/
    std::list<KeyFrame*> mKeyframeList;
    std::thread mMyThread;
    // bool shutDownFlag = false;
    // std::mutex shutDownMutex;
    std::atomic<bool> mShutdownFlag{false};
    // std::condition_variable mKeyFrameUpdated;
    std::mutex mKeyframeMutex, mUpdateMutex, mGlobalMapMutex;

    // double resolution{0.05};
    // double meank{50};
    // double thresh{1};

    /*****/
    pcl::VoxelGrid <PointT> mVoxelFilter;
    pcl::StatisticalOutlierRemoval <PointT> mStatisticalFilter;

    // std::unique_ptr<TensorRT::SemanticSeger> mSegmenter{nullptr};


private:
    // void (PointCloudMapping::*processPointCloud)(KeyFrame* kf, PointCloud& cloud) const;

    // void processPointCloudWithSeg(KeyFrame* kf, PointCloud& cloud);
    // void processPointCloudWithoutSeg(KeyFrame* kf, PointCloud& cloud);

    void generatePointCloud(KeyFrame* kf);

    void run();
};
}
