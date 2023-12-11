#include "SystemNode.h"
#include "common.hpp"

template<typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT>& cloud_in, pcl::PointCloud<PointT>& cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y +
                cloud_in.points[i].z * cloud_in.points[i].z <
            thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        ++j;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

SystemNode::SystemNode(const std::string& name) : rclcpp::Node(name)
{
    declare_parameter("ScanLine", 16);
    declare_parameter("MinimumRange", 0.1);
    declare_parameter("LineResolution", 0.06f);
    declare_parameter("PlaneResolution", 0.06f);
    get_parameter("ScanLine", mScanLines);
    get_parameter("MinimumRange", mMinimumRange);

    float line_resolution, plane_resolution;
    get_parameter("LineResolution", line_resolution);
    get_parameter("PlaneResolution", plane_resolution);

    mRawCloudSub = create_subscription<sensor_msgs::msg::PointCloud2>(
        "track_cart/Lidar/point_cloud",
        rclcpp::SensorDataQoS().reliable(),
        std::bind(&SystemNode::cloudCallBack, this, std::placeholders::_1));

    mLocalizer = std::make_shared<Localizing>();
    mMapper = std::make_shared<Mapping>(line_resolution, plane_resolution);
    mLocalizer->setMapper(mMapper);
}

void SystemNode::cloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::vector<int> scan_start_index(mScanLines, 0);
    std::vector<int> scan_end_index(mScanLines, 0);

    pcl::PointCloud<pcl::PointXYZ> input_cloud;
    pcl::fromROSMsg(*msg, input_cloud);
    std::vector<int> indices;

    pcl::removeNaNFromPointCloud(input_cloud, input_cloud, indices);
    removeClosedPointCloud(input_cloud, input_cloud, mMinimumRange);


    int cloudSize = input_cloud.points.size();
    float startOri = -std::atan2(input_cloud.points[0].y, input_cloud.points[0].x);
    float endOri = -std::atan2(input_cloud.points[cloudSize - 1].y, input_cloud.points[cloudSize - 1].x) + 2 * M_PI;

    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }

    bool halfPassed = false;
    int count = cloudSize;
    PointType point;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(mScanLines);
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = input_cloud.points[i].x;
        point.y = input_cloud.points[i].y;
        point.z = input_cloud.points[i].z;

        float angle = std::atan(point.z / std::sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        if (mScanLines == 16)
        {
            scanID = static_cast<int>((angle + 15) / 2 + 0.5);
            if (scanID > (mScanLines - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (mScanLines == 32)
        {
            scanID = static_cast<int>((angle + 92.0 / 3.0) * 3.0 / 4.0);
            if (scanID > (mScanLines - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (mScanLines == 64)
        {
            if (angle >= -8.83)
                scanID = static_cast<int>((2 - angle) * 3.0 + 0.5);
            else
                scanID = mScanLines / 2 + static_cast<int>((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Wrong scan number!");
            std::exit(EXIT_FAILURE);
        }

        float ori = -std::atan2(point.y, point.x);
        if (!halfPassed)
        {
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        float relTime = (ori - startOri) / (endOri - startOri);
        point.intensity = scanID + mScanPeriod * relTime;
        laserCloudScans[scanID].push_back(point);
    }

    cloudSize = count;

    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < mScanLines; i++)
    {
        scan_start_index[i] = cloud->size() + 5;
        *cloud += laserCloudScans[i];
        scan_end_index[i] = cloud->size() - 6;
    }

    for (int i = 5; i < cloudSize - 5; i++)
    {
        float diffX = cloud->points[i - 5].x + cloud->points[i - 4].x + cloud->points[i - 3].x +
                      cloud->points[i - 2].x + cloud->points[i - 1].x - 10 * cloud->points[i].x +
                      cloud->points[i + 1].x + cloud->points[i + 2].x + cloud->points[i + 3].x +
                      cloud->points[i + 4].x + cloud->points[i + 5].x;
        float diffY = cloud->points[i - 5].y + cloud->points[i - 4].y + cloud->points[i - 3].y +
                      cloud->points[i - 2].y + cloud->points[i - 1].y - 10 * cloud->points[i].y +
                      cloud->points[i + 1].y + cloud->points[i + 2].y + cloud->points[i + 3].y +
                      cloud->points[i + 4].y + cloud->points[i + 5].y;
        float diffZ = cloud->points[i - 5].z + cloud->points[i - 4].z + cloud->points[i - 3].z +
                      cloud->points[i - 2].z + cloud->points[i - 1].z - 10 * cloud->points[i].z +
                      cloud->points[i + 1].z + cloud->points[i + 2].z + cloud->points[i + 3].z +
                      cloud->points[i + 4].z + cloud->points[i + 5].z;

        mCloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd[i] = i;
        mCloudNeighborPicked[i] = 0;
        mCloudLabel[i] = 0;
    }

    pcl::PointCloud<PointType>::Ptr sharp_corner_cloud = std::make_shared<pcl::PointCloud<PointType>>();
    pcl::PointCloud<PointType>::Ptr less_sharp_corner_cloud = std::make_shared<pcl::PointCloud<PointType>>();
    pcl::PointCloud<PointType>::Ptr flat_surf_cloud = std::make_shared<pcl::PointCloud<PointType>>();
    pcl::PointCloud<PointType>::Ptr less_flat_surf_cloud = std::make_shared<pcl::PointCloud<PointType>>();

    for (int i = 0; i < mScanLines; i++)
    {
        if (scan_end_index[i] - scan_start_index[i] < 6) continue;
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        for (int j = 0; j < 6; j++)
        {
            int sp = scan_start_index[i] + (scan_end_index[i] - scan_start_index[i]) * j / 6;
            int ep = scan_start_index[i] + (scan_end_index[i] - scan_start_index[i]) * (j + 1) / 6 - 1;

            std::sort(cloudSortInd + sp,
                      cloudSortInd + ep + 1,
                      [&](int i, int j) -> bool { return mCloudCurvature[i] < mCloudCurvature[j]; });

            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--)
            {
                int index = cloudSortInd[k];

                if (mCloudNeighborPicked[index] == 0 && mCloudCurvature[index] > 0.1)
                {

                    largestPickedNum++;
                    if (largestPickedNum <= 2)
                    {
                        mCloudLabel[index] = 2;
                        sharp_corner_cloud->push_back(cloud->points[index]);
                        less_sharp_corner_cloud->push_back(cloud->points[index]);
                    }
                    else if (largestPickedNum <= 20)
                    {
                        mCloudLabel[index] = 1;
                        less_sharp_corner_cloud->push_back(cloud->points[index]);
                    }
                    else
                    {
                        break;
                    }

                    mCloudNeighborPicked[index] = 1;

                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = cloud->points[index + l].x - cloud->points[index + l - 1].x;
                        float diffY = cloud->points[index + l].y - cloud->points[index + l - 1].y;
                        float diffZ = cloud->points[index + l].z - cloud->points[index + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        mCloudNeighborPicked[index + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = cloud->points[index + l].x - cloud->points[index + l + 1].x;
                        float diffY = cloud->points[index + l].y - cloud->points[index + l + 1].y;
                        float diffZ = cloud->points[index + l].z - cloud->points[index + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        mCloudNeighborPicked[index + l] = 1;
                    }
                }
            }

            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++)
            {
                int index = cloudSortInd[k];

                if (mCloudNeighborPicked[index] == 0 && mCloudCurvature[index] < 0.1)
                {

                    mCloudLabel[index] = -1;
                    flat_surf_cloud->push_back(cloud->points[index]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)
                    {
                        break;
                    }

                    mCloudNeighborPicked[index] = 1;
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = cloud->points[index + l].x - cloud->points[index + l - 1].x;
                        float diffY = cloud->points[index + l].y - cloud->points[index + l - 1].y;
                        float diffZ = cloud->points[index + l].z - cloud->points[index + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        mCloudNeighborPicked[index + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = cloud->points[index + l].x - cloud->points[index + l + 1].x;
                        float diffY = cloud->points[index + l].y - cloud->points[index + l + 1].y;
                        float diffZ = cloud->points[index + l].z - cloud->points[index + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        mCloudNeighborPicked[index + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                if (mCloudLabel[k] <= 0)
                {
                    surfPointsLessFlatScan->push_back(cloud->points[k]);
                }
            }
        }

        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        *less_flat_surf_cloud += surfPointsLessFlatScanDS;
    }
    mLocalizer->insertCloud(cloud, sharp_corner_cloud, less_sharp_corner_cloud, flat_surf_cloud, less_flat_surf_cloud);
}
