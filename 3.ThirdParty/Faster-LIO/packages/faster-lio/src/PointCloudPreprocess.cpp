#include "PointCloudPreprocess.h"
#include "macro_print.h"
#include <pcl/filters/filter.h>
// #include <execution>

namespace faster_lio
{
void PointCloudPreprocess::set(LidarType lid_type, double blind, int pfilt_num)
{
    mLidarType = lid_type;
    mBlind = blind;
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

    // // pcl::PointCloud<velodyne_ros::Point> pl_orig;
    // pcl::PointCloud<pcl::PointXYZ> pl_orig;
    // pcl::fromROSMsg(*msg, pl_orig);
    // int plsize = pl_orig.points.size();
    // mOutCloud.reserve(plsize);

    // /*** These variables only works when no point timestamps given ***/
    // double omega_l = 3.61;   // scan angular velocity
    // std::vector<bool> is_first(mScanNum, true);
    // std::vector<double> yaw_fp(mScanNum, 0.0);     // yaw of first scan point
    // std::vector<float> yaw_last(mScanNum, 0.0);    // yaw of last scan point
    // std::vector<float> time_last(mScanNum, 0.0);   // last offset time
    // /*****************************************************************/

    // // if (pl_orig.points[plsize - 1].time > 0)
    // // {
    // //     mOffsetTimeGiven = true;
    // // }
    // // else
    // // {
    // //     mOffsetTimeGiven = false;
    // //     double yaw_first = std::atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
    // //     double yaw_end = yaw_first;
    // //     int layer_first = pl_orig.points[0].ring;
    // //     for (uint i = plsize - 1; i > 0; i--)
    // //     {
    // //         if (pl_orig.points[i].ring == layer_first)
    // //         {
    // //             yaw_end = std::atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
    // //             break;
    // //         }
    // //     }
    // // }

    // //-----------------
    // mOffsetTimeGiven = false;
    // double yaw_first = std::atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
    // double yaw_end = yaw_first;
    // int layer_first = pl_orig.points[0].ring;
    // for (uint i = plsize - 1; i > 0; i--)
    // {
    //     if (pl_orig.points[i].ring == layer_first)
    //     {
    //         yaw_end = std::atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
    //         break;
    //     }
    // }
    // //--------------------

    // for (int i = 0; i < plsize; i++)
    // {
    //     PointType added_pt;

    //     added_pt.normal_x = 0;
    //     added_pt.normal_y = 0;
    //     added_pt.normal_z = 0;
    //     added_pt.x = pl_orig.points[i].x;
    //     added_pt.y = pl_orig.points[i].y;
    //     added_pt.z = pl_orig.points[i].z;
    //     added_pt.intensity = pl_orig.points[i].intensity;
    //     added_pt.curvature = pl_orig.points[i].time * mTimeScale;   // curvature unit: ms

    //     if (!mOffsetTimeGiven)
    //     {
    //         int layer = pl_orig.points[i].ring;
    //         double yaw_angle = std::atan2(added_pt.y, added_pt.x) * 57.2957;

    //         if (is_first[layer])
    //         {
    //             yaw_fp[layer] = yaw_angle;
    //             is_first[layer] = false;
    //             added_pt.curvature = 0.0;
    //             yaw_last[layer] = yaw_angle;
    //             time_last[layer] = added_pt.curvature;
    //             continue;
    //         }

    //         // compute offset time
    //         if (yaw_angle <= yaw_fp[layer])
    //         {
    //             added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
    //         }
    //         else
    //         {
    //             added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
    //         }

    //         if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

    //         yaw_last[layer] = yaw_angle;
    //         time_last[layer] = added_pt.curvature;
    //     }

    //     if (i % mPointfilterNum == 0)
    //     {
    //         if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (mBlind * mBlind))
    //         {
    //             mOutCloud.points.push_back(added_pt);
    //         }
    //     }
    // }

    //-----------------------------------------------------------------------

    // std::vector<int> scan_start_index(mScanNum, 0);
    // std::vector<int> scan_end_index(mScanNum, 0);
    /*** These variables only works when no point timestamps given ***/
    double omega_l = 3.61;   // scan angular velocity
    std::vector<bool> is_first(mScanNum, true);
    std::vector<double> yaw_fp(mScanNum, 0.0);     // yaw of first scan point
    std::vector<float> yaw_last(mScanNum, 0.0);    // yaw of last scan point
    std::vector<float> time_last(mScanNum, 0.0);   // last offset time

    pcl::PointCloud<pcl::PointXYZ> input_cloud;
    pcl::fromROSMsg(*msg, input_cloud);
    // std::vector<int> indices;

    int cloud_size = input_cloud.points.size();
    float yaw_first_ = -std::atan2(input_cloud.points[0].y, input_cloud.points[0].x);
    float yaw_end_ = -std::atan2(input_cloud.points[cloud_size - 1].y, input_cloud.points[cloud_size - 1].x) + 2 * M_PI;

    if (yaw_end_ - yaw_first_ > 3 * M_PI)
    {
        yaw_end_ -= 2 * M_PI;
    }
    else if (yaw_end_ - yaw_first_ < M_PI)
    {
        yaw_end_ += 2 * M_PI;
    }

    bool half_passed = false;
    int count = cloud_size;
    PointType point;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(mScanNum);
    for (int i = 0; i < cloud_size; i++)
    {
        point.x = input_cloud.points[i].x;
        point.y = input_cloud.points[i].y;
        point.z = input_cloud.points[i].z;

        float yaw_angle = std::atan(point.z / std::sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scan_id = 0;

        switch (mScanNum)
        {
            case 16:
                scan_id = static_cast<int>((yaw_angle + 15) / 2 + 0.5);
                if (scan_id > (mScanNum - 1) || scan_id < 0)
                {
                    count--;
                    continue;
                }
                break;
            case 32:
                scan_id = static_cast<int>((yaw_angle + 92.0 / 3.0) * 3.0 / 4.0);
                if (scan_id > (mScanNum - 1) || scan_id < 0)
                {
                    count--;
                    continue;
                }
                break;
            case 64:
                if (yaw_angle >= -8.83)
                {
                    scan_id = static_cast<int>((2 - yaw_angle) * 3.0 + 0.5);
                }
                else
                {
                    scan_id = mScanNum / 2 + static_cast<int>((-8.83 - yaw_angle) * 2.0 + 0.5);
                }
                if (yaw_angle > 2 || yaw_angle < -24.33 || scan_id > 50 || scan_id < 0)
                {
                    count--;
                    continue;
                }
                break;
            default:
                throw std::runtime_error("Wrong Scan Lines!");
                break;
        }

        float ori = -std::atan2(point.y, point.x);

        if (!half_passed)
        {
            if (ori < yaw_first_ - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > yaw_first_ + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - yaw_first_ > M_PI)
            {
                half_passed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < yaw_end_ - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > yaw_end_ + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        //---
        if (is_first[scan_id])
        {
            yaw_fp[scan_id] = yaw_angle;
            is_first[scan_id] = false;
            point.curvature = 0.0;
            yaw_last[scan_id] = yaw_angle;
            time_last[scan_id] = point.curvature;
            continue;
        }
        /* Compute offset time */
        if (yaw_angle <= yaw_fp[scan_id])
        {
            point.curvature = (yaw_fp[scan_id] - yaw_angle) / omega_l;
        }
        else
        {
            point.curvature = (yaw_fp[scan_id] - yaw_angle + 360.0) / omega_l;
        }

        if (point.curvature < time_last[scan_id]) point.curvature += 360.0 / omega_l;

        yaw_last[scan_id] = yaw_angle;
        time_last[scan_id] = point.curvature;
        //---

        float real_time = (ori - yaw_first_) / (yaw_end_ - yaw_first_);
        point.intensity = scan_id + 0.1 * real_time;
        laserCloudScans[scan_id].push_back(point);
    }

    for (int i = 0; i < mScanNum; i++)
    {
        mOutCloud += laserCloudScans[i];
    }
}

}   // namespace faster_lio
