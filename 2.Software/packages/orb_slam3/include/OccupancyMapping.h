#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

// #include <pcl/io/pcd_io.h>
// #include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/conditional_removal.h>           //条件滤波器头文件
#include <pcl/filters/passthrough.h>                   //直通滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>        //半径滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h>   //统计滤波器头文件
// #include <pcl/filters/voxel_grid.h>                    //体素滤波器头文件
// #include <pcl/point_types.h>

class OccupancyMapping : public rclcpp::Node
{
public:
    explicit OccupancyMapping(const std::string& name);
    OccupancyMapping(const OccupancyMapping&) = delete;
    OccupancyMapping& operator=(const OccupancyMapping&) = delete;   // 禁止赋值运算符
    ~OccupancyMapping();

    void insertPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& cloud)
    {
        std::unique_lock<std::mutex> lock(mUpdateMapMutex);
        mPointCloud = cloud;
        mUpdateMapFlag = true;
        mUpdataMapCondition.notify_one();
    }

    void run();

private:
    /*** Parameters ***/
    float mThreZmin{0.2f}, mThreZmax{2.0f};
    bool mPassThroughFlag{false};
    float mThreRadius{0.5f};
    double mMapResolution;
    uint16_t mThresPointCount{10};

    /*** Multi-Threaded ***/
    std::thread mMyThread;
    std::atomic<bool> mShutdownFlag{false};
    std::condition_variable mUpdataMapCondition;
    bool mUpdateMapFlag{false};
    std::mutex mUpdateMapMutex;


    /*** Containers ***/
    pcl::PointCloud<pcl::PointXYZRGB> mPointCloud;

    /*** Tools ***/
    pcl::PassThrough<pcl::PointXYZRGB> mPassthroughFilter;                     // 创建滤波器对象
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> mRadiusOutlierRemovalFilter;   // 创建滤波器

    /*** RCLCPP ***/
    nav_msgs::msg::OccupancyGrid mOccupancyGridMsg;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mOccupancyMapPublisher;

private:
    void toOccupancyGrid();
};