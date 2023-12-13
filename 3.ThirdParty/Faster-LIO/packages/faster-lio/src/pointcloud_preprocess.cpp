#include "pointcloud_preprocess.h"
// #include <glog/logging.h>
#include "macro_print.h"
#include <execution>

namespace faster_lio
{
void PointCloudPreprocess::set(LidarType lid_type, double bld, int pfilt_num)
{
    mLidarType = lid_type;
    mBlind = bld;
    mPointfilterNum = pfilt_num;
}

void PointCloudPreprocess::process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
                                   PointCloudType::Ptr& pcl_out)
{
    switch (mLidarType)
    {
        case LidarType::OUST64:
            Oust64Handler(msg);
            break;

        case LidarType::VELO32:
            VelodyneHandler(msg);
            break;

        default:
            THROW_ERROR("Error LiDAR Type");
            break;
    }
    *pcl_out = mOutCloud;
}

void PointCloudPreprocess::Oust64Handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    mOutCloud.clear();
    mFullCloud.clear();
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.size();
    mOutCloud.reserve(plsize);

    for (int i = 0; i < pl_orig.points.size(); i++)
    {
        if (i % mPointfilterNum != 0) continue;

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;

        if (range < (mBlind * mBlind)) continue;

        Eigen::Vector3d pt_vec;
        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.curvature = pl_orig.points[i].t / 1e6;   // curvature unit: ms

        mOutCloud.points.push_back(added_pt);
    }
}

void PointCloudPreprocess::VelodyneHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    mOutCloud.clear();
    mFullCloud.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    mOutCloud.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 3.61;   // scan angular velocity
    std::vector<bool> is_first(mScanNum, true);
    std::vector<double> yaw_fp(mScanNum, 0.0);     // yaw of first scan point
    std::vector<float> yaw_last(mScanNum, 0.0);    // yaw of last scan point
    std::vector<float> time_last(mScanNum, 0.0);   // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0)
    {
        mOffsetTimeGiven = true;
    }
    else
    {
        mOffsetTimeGiven = false;
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
        double yaw_end = yaw_first;
        int layer_first = pl_orig.points[0].ring;
        for (uint i = plsize - 1; i > 0; i--)
        {
            if (pl_orig.points[i].ring == layer_first)
            {
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
                break;
            }
        }
    }

    for (int i = 0; i < plsize; i++)
    {
        PointType added_pt;

        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time * mTimeScale;   // curvature unit: ms

        if (!mOffsetTimeGiven)
        {
            int layer = pl_orig.points[i].ring;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer])
            {
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                added_pt.curvature = 0.0;
                yaw_last[layer] = yaw_angle;
                time_last[layer] = added_pt.curvature;
                continue;
            }

            // compute offset time
            if (yaw_angle <= yaw_fp[layer])
            {
                added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
            }
            else
            {
                added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }

            if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

            yaw_last[layer] = yaw_angle;
            time_last[layer] = added_pt.curvature;
        }

        if (i % mPointfilterNum == 0)
        {
            if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (mBlind * mBlind))
            {
                mOutCloud.points.push_back(added_pt);
            }
        }
    }
}

}   // namespace faster_lio
