#include "OccupancyMapping.h"
#include <opencv2/core.hpp>

OccupancyMapping::OccupancyMapping(const std::string& name) : rclcpp::Node(name)
{
    /******************************** Configure parameters ****************************************/
    this->declare_parameter("OccupancyMap.Resolution", 0.05f);
    this->declare_parameter("OccupancyMap.FrameId", "map");
    this->declare_parameter("PointCloudFilter.PassThrough.Axis", "y");
    this->declare_parameter("PointCloudFilter.PassThrough.LimitMin", -2.0f);
    this->declare_parameter("PointCloudFilter.PassThrough.LimitMax", -0.2f);
    this->declare_parameter("PointCloudFilter.RadiusRemoval.Radius", 0.5);
    this->declare_parameter("PointCloudFilter.RadiusRemoval.MinNeighbors", 10);
    float map_resolution;
    double radius_removal_radius;
    int min_neighbor;
    float pass_through_min, pass_through_max;
    this->get_parameter("OccupancyMap.Resolution", map_resolution);
    this->get_parameter("PointCloudFilter.PassThrough.LimitMin", pass_through_min);
    this->get_parameter("PointCloudFilter.PassThrough.LimitMax", pass_through_max);
    this->get_parameter("PointCloudFilter.RadiusRemoval.Radius", radius_removal_radius);
    this->get_parameter("PointCloudFilter.RadiusRemoval.MinNeighbors", min_neighbor);
    /**********************************************************************************************/

    mOccupancyMapPublisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("orb_slam3/occupancy_map",
                                                                                  rclcpp::ParametersQoS().reliable());

    /*** Configure filters ***/
    mPassthroughFilter.setFilterFieldName("y");
    mPassthroughFilter.setFilterLimits(pass_through_min, pass_through_max);
    mPassthroughFilter.setFilterLimitsNegative(false);   // true表示保留滤波范围外，false表示保留范围内
    mRadiusOutlierRemovalFilter.setRadiusSearch(radius_removal_radius);
    mRadiusOutlierRemovalFilter.setMinNeighborsInRadius(min_neighbor);

    mOccupancyGridMsg.info.resolution = map_resolution;
    mOccupancyGridMsg.header.frame_id = "map";

    mMyThread = std::thread(&OccupancyMapping::run, this);
}

OccupancyMapping::~OccupancyMapping()
{
    mShutdownFlag = true;
    mMyThread.join();
}

void OccupancyMapping::toOccupancyGrid()
{
    mOccupancyGridMsg.info.map_load_time = this->now();
    double x_min, x_max, y_min, y_max;
    double z_max_grey_rate = 0.05;
    double z_min_grey_rate = 0.95;

    double k_line = (z_max_grey_rate - z_min_grey_rate) / (mThreZmax - mThreZmin);
    double b_line = (mThreZmax * z_min_grey_rate - mThreZmin * z_max_grey_rate) / (mThreZmax - mThreZmin);

    for (int i = 0; i < mPointCloud.size() - 1; i++)
    {
        if (i == 0)
        {
            x_min = x_max = mPointCloud.points[i].x;
            y_min = y_max = mPointCloud.points[i].y;
        }

        double x = mPointCloud.points[i].x;
        double y = mPointCloud.points[i].y;

        if (x < x_min) x_min = x;
        if (x > x_max) x_max = x;

        if (y < y_min) y_min = y;
        if (y > y_max) y_max = y;
    }
    // origin的确定
    mOccupancyGridMsg.info.origin.position.x = x_min;
    mOccupancyGridMsg.info.origin.position.y = y_min;
    mOccupancyGridMsg.info.origin.position.z = 0.0;
    mOccupancyGridMsg.info.origin.orientation.x = 0.0;
    mOccupancyGridMsg.info.origin.orientation.y = 0.0;
    mOccupancyGridMsg.info.origin.orientation.z = 0.0;
    mOccupancyGridMsg.info.origin.orientation.w = 1.0;
    // 设置栅格地图大小
    mOccupancyGridMsg.info.width = static_cast<uint32_t>((x_max - x_min) / mMapResolution);
    mOccupancyGridMsg.info.height = static_cast<uint32_t>((y_max - y_min) / mMapResolution);
    // 实际地图中某点坐标为(x,y)，对应栅格地图中坐标为[x*map.info.width+y]
    mOccupancyGridMsg.data.resize(mOccupancyGridMsg.info.width * mOccupancyGridMsg.info.height);
    mOccupancyGridMsg.data.assign(mOccupancyGridMsg.info.width * mOccupancyGridMsg.info.height, 0);

    for (int iter = 0; iter < mPointCloud.size(); iter++)
    {
        int i = static_cast<int>((mPointCloud.points[iter].x - x_min) / mMapResolution);
        if (i < 0 || i >= mOccupancyGridMsg.info.width) continue;

        int j = static_cast<int>((mPointCloud.points[iter].y - y_min) / mMapResolution);
        if (j < 0 || j >= mOccupancyGridMsg.info.height - 1) continue;
        // 栅格地图的占有概率[0,100]，这里设置为占据
        mOccupancyGridMsg.data[i + j * mOccupancyGridMsg.info.width] = 100;
    }
}

void OccupancyMapping::run()
{
    while (!mShutdownFlag)
    {
        {
            std::unique_lock<std::mutex> lock(mUpdateMapMutex);
            mUpdataMapCondition.wait(lock, [&] { return mUpdateMapFlag; });
            mUpdateMapFlag = false;
        }

        if (mPointCloud.empty())
        {
            RCLCPP_WARN(this->get_logger(), "The input point cloud is empty!");
            continue;
        }

        mPassthroughFilter.setInputCloud(mPointCloud.makeShared());
        mPassthroughFilter.filter(mPointCloud);
        mRadiusOutlierRemovalFilter.setInputCloud(mPointCloud.makeShared());
        mRadiusOutlierRemovalFilter.filter(mPointCloud);

        void toOccupancyGrid();

        mOccupancyGridMsg.header.stamp = this->now();
        mOccupancyMapPublisher->publish(mOccupancyGridMsg);
    }
}
