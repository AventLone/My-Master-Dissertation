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

void PointCloudMapping::shutdown()
{
    mShutdownFlag = true;
    mMyThread.join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf)
{
    if (kf->imgRGB.empty()) return;
    std::unique_lock<std::mutex> lock(mKeyframeMutex);
    detectGreenBlue(kf->imgRGB);

    mKeyframeList.push_back(kf);
    if (mKeyframeList.size() > 35) mKeyframeList.pop_front();
}


/*** Generate a pointcloud from a key frame. ***/
void PointCloudMapping::generatePointCloud(KeyFrame* kf)
{
    PointCloud point_cloud;
    PointT point;
    for (int v = 0; v < kf->imgDepth.rows; v += 3)
    {
        for (int u = 0; u < kf->imgDepth.cols; u += 3)
        {
            auto d = kf->imgDepth.ptr<unsigned short>(v)[u];
            if (d < 100 || d > 8000) continue;   // Remove abnormal points

            point.z = static_cast<float>(d) / 1000.0f;
            point.x = (u - kf->cx) * point.z / kf->fx;
            point.y = (v - kf->cy) * point.z / kf->fy;

            point.r = kf->imgRGB.ptr<uchar>(v)[u * 3];
            point.g = kf->imgRGB.ptr<uchar>(v)[u * 3 + 1];
            point.b = kf->imgRGB.ptr<uchar>(v)[u * 3 + 2];

            point_cloud.points.push_back(point);
        }
    }

    mVoxelFilter.setInputCloud(point_cloud.makeShared());
    mVoxelFilter.filter(point_cloud);
    // mStatisticalFilter.setInputCloud(point_cloud.makeShared());
    // mStatisticalFilter.filter(point_cloud);

    kf->mPointCloud = point_cloud;
}

/*** Loop closing to update the globe map ***/
void PointCloudMapping::updatePointCloud(Map& cur_map)
{
    // std::unique_lock<std::mutex> lock(mUpdateMutex);
    mIsUpdating = true;
    mCurrentKFs = cur_map.GetAllKeyFrames();

    PointCloud tmp_global_map, cur_pointcloud;
    for (const auto& kf : mCurrentKFs)
    {
        if (!mIsUpdating)
        {
            return;
        }
        if (!kf->isBad() && !kf->mPointCloud.empty())
        {
            pcl::transformPointCloud(kf->mPointCloud, cur_pointcloud, kf->GetPoseInverse().matrix());
            tmp_global_map += cur_pointcloud;
        }
    }

    std::cout << "Point cloud update finished." << std::endl;
    {
        std::unique_lock<std::mutex> lock(mGlobalMapMutex);
        mGlobalMap = tmp_global_map;
    }
    mIsUpdating = false;
}

void PointCloudMapping::run()
{
    while (!mShutdownFlag)
    {
        std::list<KeyFrame*> temp_keyframe_list;

        {
            std::unique_lock<std::mutex> lock(mKeyframeMutex);
            if (mKeyframeList.empty())
            {
                continue;
            }
            else
            {
                temp_keyframe_list = mKeyframeList;
                mKeyframeList.clear();
            }
        }

        for (const auto& pKF : temp_keyframe_list)
        {
            if (pKF->isBad()) continue;

            generatePointCloud(pKF);

            PointCloud point_cloud;
            pcl::transformPointCloud(pKF->mPointCloud, point_cloud, pKF->GetPoseInverse().matrix());

            {
                std::unique_lock<std::mutex> lock(mGlobalMapMutex);
                mGlobalMap += point_cloud;
            }
        }
    }
}
}   // namespace ORB_SLAM3