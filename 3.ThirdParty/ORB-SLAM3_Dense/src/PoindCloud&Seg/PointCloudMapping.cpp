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
        fs_settings["CloudMap.Resolution"] >> resolution;
        fs_settings["CloudMap.Mode"] >> mMapMode;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        PRINT_WARN("Failed to load point cloud resolution so set it to 0.05!");
        resolution = 0.05;
    }
    fs_settings.release();

    switch (mMapMode)
    {
        case COLORFUL:
            mColorfunlMap = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
            break;
        case SEMANTIC:
            mSemanticMap = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        default:
            break;
    }

    mVoxelFilter_color.setLeafSize(resolution, resolution, resolution);
    mVoxelFilter_semantic.setLeafSize(resolution, resolution, resolution);
    // mStatisticalFilter.setMeanK(meank);
    // mStatisticalFilter.setStddevMulThresh(thresh);

    mMyThread = std::thread(&PointCloudMapping::run, this);
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf)
{
    if (kf->imgRGB.empty()) return;
    {
        std::unique_lock<std::mutex> lock(mKeyframeMutex);
        if (mMapMode == SEMANTIC)
        {
            detectGreenBlue(kf->imgRGB);
        }

        mKeyFrameBuffer.push(kf);
        if (mKeyFrameBuffer.size() > 35) mKeyFrameBuffer.pop();
        mKeyFrameUpdate.notify_one();
    }
}

/*** Generate a pointcloud from a key frame. ***/
void PointCloudMapping::generatePointCloud(KeyFrame* kf)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB color_point;
    pcl::PointCloud<pcl::PointXYZI>::Ptr semantic_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI semantic_point;

    if (mMapMode == SEMANTIC)
    {
        for (int v = 0; v < kf->imgDepth.rows; v += 9)
        {
            for (int u = 0; u < kf->imgDepth.cols; u += 9)
            {
                auto d = kf->imgDepth.ptr<uint16_t>(v)[u];
                if (d < 10 || d > 10000) continue;   // Remove abnormal points

                semantic_point.z = static_cast<float>(d) / 1000.0f;
                semantic_point.x = (u - kf->cx) * semantic_point.z / kf->fx;
                semantic_point.y = (v - kf->cy) * semantic_point.z / kf->fy;

                // semantic_point.r = kf->imgRGB.ptr<uchar>(v)[u * 3];

                semantic_cloud->points.push_back(semantic_point);
            }
        }
        semantic_cloud->height = 1;
        semantic_cloud->width = color_cloud->points.size();
        semantic_cloud->is_dense = false;

        mVoxelFilter_semantic.setInputCloud(semantic_cloud);
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        mVoxelFilter_semantic.filter(*temp_cloud);

        kf->mSemanticCloud = temp_cloud;
    }
    else
    {
        for (int v = 0; v < kf->imgDepth.rows; v += 9)
        {
            for (int u = 0; u < kf->imgDepth.cols; u += 9)
            {
                auto d = kf->imgDepth.ptr<uint16_t>(v)[u];
                if (d < 10 || d > 10000) continue;   // Remove abnormal points

                color_point.z = static_cast<float>(d) / 1000.0f;
                color_point.x = (u - kf->cx) * color_point.z / kf->fx;
                color_point.y = (v - kf->cy) * color_point.z / kf->fy;

                color_point.r = kf->imgRGB.ptr<uchar>(v)[u * 3];
                color_point.g = kf->imgRGB.ptr<uchar>(v)[u * 3 + 1];
                color_point.b = kf->imgRGB.ptr<uchar>(v)[u * 3 + 2];

                color_cloud->points.push_back(color_point);
            }
        }
        color_cloud->height = 1;
        color_cloud->width = color_cloud->points.size();
        color_cloud->is_dense = false;

        mVoxelFilter_color.setInputCloud(color_cloud);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        mVoxelFilter_color.filter(*temp_cloud);

        kf->mColorCloud = temp_cloud;
    }
}

/*** Loop closing to update the globe map ***/
void PointCloudMapping::updatePointCloud(Map& cur_map)
{
    mIsUpdating = true;
    mCurrentKFs = cur_map.GetAllKeyFrames();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_global_map(new pcl::PointCloud<pcl::PointXYZRGB>),
        cur_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& kf : mCurrentKFs)
    {
        if (!mIsUpdating)
        {
            return;
        }
        if (!kf->isBad() && !kf->mColorCloud->empty())
        {
            pcl::transformPointCloud(*(kf->mColorCloud), *cur_pointcloud, kf->GetPoseInverse().matrix());
            *tmp_global_map += *cur_pointcloud;
        }
    }

    PRINT_INFO("Point cloud update finished.");
    {
        std::unique_lock<std::mutex> lock(mGlobalMapMutex);
        mColorfunlMap = tmp_global_map;
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

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*(key_frame->mColorCloud), *temp_cloud, key_frame->GetPoseInverse().matrix());

        {
            std::unique_lock<std::mutex> lock(mGlobalMapMutex);
            *mColorfunlMap += *temp_cloud;
        }
    }
}
}   // namespace ORB_SLAM3