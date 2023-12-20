#include "PointCloudMapping.h"
#include "macro_print.h"
#include <pcl/common/transforms.h>

namespace ORB_SLAM3
{
static void detectGreenBlue(cv::Mat& src)
{
    cv::Mat green_img(src.size(), CV_8UC3, cv::Scalar(0, 200, 0));
    cv::Mat blue_img(src.size(), CV_8UC3, cv::Scalar(250, 0, 0));

    cv::Mat blue_mask, green_mask;
    cv::inRange(src, cv::Scalar(160, 0, 0), cv::Scalar(255, 100, 100), blue_mask);
    cv::inRange(src, cv::Scalar(0, 50, 0), cv::Scalar(100, 255, 100), green_mask);

    src = cv::Mat(src.size(), CV_8UC3, cv::Scalar(210, 210, 210));

    green_img.copyTo(src, green_mask);
    blue_img.copyTo(src, blue_mask);
}

PointCloudMapping::PointCloudMapping(const std::string& setting_file, const std::string& seg_net_file)
{
    cv::FileStorage fs_settings(setting_file, cv::FileStorage::READ);
    if (!fs_settings.isOpened())
    {
        THROW_ERROR("Failed to open settings file at: " + setting_file);
    }
    float resolution;
    try
    {
        fs_settings["MapResolution.PointCloud"] >> resolution;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        PRINT_WARN("Failed to load point cloud resolution so set it to 0.05!");
        resolution = 0.05;
    }
    fs_settings.release();

    mVoxelFilter.setLeafSize(resolution, resolution, resolution);
    // mStatisticalFilter.setMeanK(meank);
    // mStatisticalFilter.setStddevMulThresh(thresh);

    mMyThread = std::thread(&PointCloudMapping::run, this);
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf)
{
    if (kf->imgRGB.empty()) return;
    {
        std::unique_lock<std::mutex> lock(mKeyframeMutex);
        // detectGreenBlue(kf->imgRGB);

        mKeyFrameBuffer.push(kf);
        if (mKeyFrameBuffer.size() > 35) mKeyFrameBuffer.pop();
        mKeyFrameUpdate.notify_one();
    }
}

/*** Generate a pointcloud from a key frame. ***/
void PointCloudMapping::generatePointCloud(KeyFrame* kf)
{
    PointCloud::Ptr point_cloud(new PointCloud);
    PointT point;
    for (int v = 0; v < kf->imgDepth.rows; v += 9)
    {
        for (int u = 0; u < kf->imgDepth.cols; u += 9)
        {
            auto d = kf->imgDepth.ptr<uint16_t>(v)[u];
            if (d < 10 || d > 10000) continue;   // Remove abnormal points

            point.z = static_cast<float>(d) / 1000.0f;
            point.x = (u - kf->cx) * point.z / kf->fx;
            point.y = (v - kf->cy) * point.z / kf->fy;

            point.r = kf->imgRGB.ptr<uchar>(v)[u * 3];
            point.g = kf->imgRGB.ptr<uchar>(v)[u * 3 + 1];
            point.b = kf->imgRGB.ptr<uchar>(v)[u * 3 + 2];

            point_cloud->points.push_back(point);
        }
    }
    point_cloud->height = 1;
    point_cloud->width = point_cloud->points.size();
    point_cloud->is_dense = false;

    mVoxelFilter.setInputCloud(point_cloud);
    PointCloud::Ptr temp_cloud(new PointCloud);
    mVoxelFilter.filter(*temp_cloud);

    kf->mPointCloud = temp_cloud;
}

/*** Loop closing to update the globe map ***/
void PointCloudMapping::updatePointCloud(Map& cur_map)
{
    mIsUpdating = true;
    mCurrentKFs = cur_map.GetAllKeyFrames();

    PointCloud::Ptr tmp_global_map(new PointCloud), cur_pointcloud(new PointCloud);
    for (const auto& kf : mCurrentKFs)
    {
        if (!mIsUpdating)
        {
            return;
        }
        if (!kf->isBad() && !kf->mPointCloud->empty())
        {
            pcl::transformPointCloud(*(kf->mPointCloud), *cur_pointcloud, kf->GetPoseInverse().matrix());
            *tmp_global_map += *cur_pointcloud;
        }
    }

    PRINT_INFO("Point cloud update finished.");
    {
        std::unique_lock<std::mutex> lock(mGlobalMapMutex);
        mGlobalMap = tmp_global_map;
    }
    mIsUpdating = false;
}

void PointCloudMapping::run()
{
    for (;;)
    {
        KeyFrame* key_frame;
        {
            std::unique_lock<std::mutex> lock(mKeyframeMutex);
            mKeyFrameUpdate.wait(lock, [this]() { return mKeyFrameBuffer.size() > 0 || mShutdown; });
            if (mShutdown) break;
            if (mIsUpdating) continue;

            key_frame = mKeyFrameBuffer.front();
            mKeyFrameBuffer.pop();
        }

        if (key_frame->isBad()) continue;

        generatePointCloud(key_frame);

        PointCloud::Ptr temp_cloud(new PointCloud);
        pcl::transformPointCloud(*(key_frame->mPointCloud), *temp_cloud, key_frame->GetPoseInverse().matrix());

        {
            std::unique_lock<std::mutex> lock(mGlobalMapMutex);
            *mGlobalMap += *temp_cloud;
        }
    }
}
}   // namespace ORB_SLAM3