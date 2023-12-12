#ifndef FASTER_LIO_POINTCLOUD_PROCESSING_H
#define FASTER_LIO_POINTCLOUD_PROCESSING_H

// #include <livox_ros_driver/CustomMsg.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common_lib.h"

namespace velodyne_ros
{
struct EIGEN_ALIGN16 Point
{
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}   // namespace velodyne_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                      (float, time, time)(std::uint16_t, ring, ring))
// clang-format on

namespace ouster_ros
{
struct EIGEN_ALIGN16 Point
{
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}   // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                  (float, x, x)
                                      (float, y, y)
                                      (float, z, z)
                                      (float, intensity, intensity)
                                      // use std::uint32_t to avoid conflicting with pcl::uint32_t
                                  (std::uint32_t, t, t)
                                  (std::uint16_t, reflectivity, reflectivity)
                                  (std::uint8_t, ring, ring)
                                  (std::uint16_t, ambient, ambient)
                                  (std::uint32_t, range, range))
// clang-format on

namespace faster_lio
{
enum class LidarType
{
    AVIA = 1,
    VELO32,
    OUST64
};

/**
 * point cloud preprocess
 * just unify the point format from livox/velodyne to PCL
 */
class PointCloudPreprocess
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointCloudPreprocess() = default;
    ~PointCloudPreprocess() = default;

    /// processors
    // void process(const livox_ros_driver::CustomMsg::ConstPtr& msg, PointCloudType::Ptr& pcl_out);
    void process(const sensor_msgs::msg::PointCloud2::ConstPtr& msg, PointCloudType::Ptr& pcl_out);
    void set(LidarType lid_type, double bld, int pfilt_num);

    /* accessors */
    double& blind()
    {
        return mBlind;
    }
    int& scanNum()
    {
        return mScanNum;
    }
    int& pointFilterNum()
    {
        return mPointfilterNum;
    }
    bool& featureEnabled()
    {
        return mFeatureEnabled;
    }
    float& timeScale()
    {
        return mTimeScale;
    }
    LidarType getLidarType() const
    {
        return mLidarType;
    }
    void setLidarType(LidarType lt)
    {
        mLidarType = lt;
    }

private:
    // void AviaHandler(const livox_ros_driver::CustomMsg::ConstPtr& msg);
    void Oust64Handler(const sensor_msgs::msg::PointCloud2::ConstPtr& msg);
    void VelodyneHandler(const sensor_msgs::msg::PointCloud2::ConstPtr& msg);

    PointCloudType mFullCloud, mOutCloud;

    LidarType mLidarType = LidarType::AVIA;
    bool mFeatureEnabled = false;
    int mPointfilterNum = 1;
    int mScanNum = 6;
    double mBlind = 0.01;
    float mTimeScale = 1e-3;
    bool mOffsetTimeGiven = false;
};
}   // namespace faster_lio

#endif
