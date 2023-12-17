#include "Mapping.h"
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


Mapping::Mapping(float line_resolution, float plane_resolution) : rclcpp::Node("aloam_mapping")
{
    mCornerCloudArray.reserve(mCloudNum);
    mSurfCloudArray.reserve(mCloudNum);
    for (int i = 0; i < mCloudNum; ++i)
    {
        mCornerCloudArray[i].reset(new pcl::PointCloud<PointType>());
        mSurfCloudArray[i].reset(new pcl::PointCloud<PointType>());
    }

    mCornerDownSizeFilter.setLeafSize(line_resolution, line_resolution, line_resolution);
    mSurfDownSizeFilter.setLeafSize(plane_resolution, plane_resolution, plane_resolution);

    mTfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    mSurroundCloudPub =
        create_publisher<sensor_msgs::msg::PointCloud2>("aloam/surround_cloud", rclcpp::SensorDataQoS().best_effort());
    mCloudMapPub =
        create_publisher<sensor_msgs::msg::PointCloud2>("aloam/cloud_map", rclcpp::SensorDataQoS().best_effort());
    mOdomPub = create_publisher<nav_msgs::msg::Odometry>("aloam/odom", rclcpp::SensorDataQoS().best_effort());
    mPathPub = create_publisher<nav_msgs::msg::Path>("aloam/path", rclcpp::SensorDataQoS().best_effort());

    mThread = std::thread(&Mapping::run, this);
}

void Mapping::insertCloudAndOdom(const pcl::PointCloud<pcl::PointXYZI>::Ptr& full_cloud,
                                 const pcl::PointCloud<pcl::PointXYZI>::Ptr& corner_cloud,
                                 const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_cloud,
                                 const Eigen::Quaterniond& orientation, const Eigen::Vector3d& position)
{
    std::unique_lock<std::mutex> lock(mBufferMutex);

    mFullCloudBuffer.emplace(full_cloud);
    mCornerCloudBuffer.emplace(corner_cloud);
    mSurfCloudBuffer.emplace(surf_cloud);
    mOrientationBuffer.emplace(orientation);
    mPositionBuffer.emplace(position);
    if (mCornerCloudBuffer.size() > 30)
    {
        RCLCPP_WARN(get_logger(), "Cloud buffers are full, have to pop out.");
        mFullCloudBuffer.pop();
        mCornerCloudBuffer.pop();
        mSurfCloudBuffer.pop();
        mOrientationBuffer.pop();
        mPositionBuffer.pop();
    }
    mCondition.notify_one();
}

void Mapping::publishMsgs(Eigen::Quaterniond orientation, Eigen::Vector3d position,
                          pcl::PointCloud<PointType>::Ptr surround_cloud, pcl::PointCloud<PointType>::Ptr cloud_map)
{
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = "cloud_map";
    odom_msg.child_frame_id = "odom";
    odom_msg.header.stamp = this->now();
    odom_msg.pose.pose.orientation.x = orientation.x();
    odom_msg.pose.pose.orientation.y = orientation.y();
    odom_msg.pose.pose.orientation.z = orientation.z();
    odom_msg.pose.pose.orientation.w = orientation.w();
    odom_msg.pose.pose.position.x = position.x();
    odom_msg.pose.pose.position.y = position.y();
    odom_msg.pose.pose.position.z = position.z();
    mOdomPub->publish(odom_msg);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = odom_msg.header;
    pose_msg.pose = odom_msg.pose.pose;
    mPath.header.stamp = odom_msg.header.stamp;
    mPath.header.frame_id = "cloud_map";
    mPath.poses.push_back(pose_msg);
    mPathPub->publish(mPath);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id = "cloud_map";
    tf_msg.child_frame_id = "odom";
    tf_msg.header.stamp = odom_msg.header.stamp;
    tf_msg.transform.translation = tf2::toMsg(tf2::Vector3(position(0), position(1), position(2)));
    tf_msg.transform.rotation =
        tf2::toMsg(tf2::Quaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w()));
    mTfBroadcaster->sendTransform(tf_msg);

    // sensor_msgs::msg::PointCloud2 surround_cloud_msg;
    // pcl::toROSMsg(*surround_cloud, surround_cloud_msg);
    // surround_cloud_msg.header.stamp = this->now();
    // surround_cloud_msg.header.frame_id = "camera_init";
    // mSurroundCloudPub->publish(surround_cloud_msg);

    sensor_msgs::msg::PointCloud2 cloud_map_msg;
    pcl::toROSMsg(*cloud_map, cloud_map_msg);
    cloud_map_msg.header.stamp = this->now();
    cloud_map_msg.header.frame_id = "cloud_map";
    mCloudMapPub->publish(cloud_map_msg);
}


void Mapping::run()
{
    for (;;)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr full_cloud, corner_cloud, surf_cloud;

        {
            std::unique_lock<std::mutex> lock(mBufferMutex);
            mCondition.wait(lock, [this]() { return mFullCloudBuffer.size() > 0 || mShutDown; });
            if (mShutDown) break;

            full_cloud = mFullCloudBuffer.front();
            mFullCloudBuffer.pop();

            corner_cloud = mCornerCloudBuffer.front();
            mCornerCloudBuffer.pop();

            surf_cloud = mSurfCloudBuffer.front();
            mSurfCloudBuffer.pop();

            q_wodom_curr = mOrientationBuffer.front();
            mOrientationBuffer.pop();

            t_wodom_curr = mPositionBuffer.front();
            mPositionBuffer.pop();
        }

        transformAssociateToMap();

        int center_cubeI = static_cast<int>((t_w_curr.x() + 25.0) / 50.0) + mCloudCenWidth;
        int center_cubeJ = static_cast<int>((t_w_curr.y() + 25.0) / 50.0) + mCloudCenHeight;
        int center_cubeK = static_cast<int>((t_w_curr.z() + 25.0) / 50.0) + mCloudCenDepth;

        if (t_w_curr.x() + 25.0 < 0) --center_cubeI;
        if (t_w_curr.y() + 25.0 < 0) --center_cubeJ;
        if (t_w_curr.z() + 25.0 < 0) --center_cubeK;

        while (center_cubeI < 3)
        {
            for (int j = 0; j < mCloudHeight; ++j)
            {
                for (int k = 0; k < mCloudDepth; ++k)
                {
                    int i = mCloudWidth - 1;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                        mCornerCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                        mSurfCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k];
                    for (; i >= 1; --i)
                    {
                        mCornerCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] =
                            mCornerCloudArray[i - 1 + mCloudWidth * j + mCloudWidth * mCloudHeight * k];
                        mSurfCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] =
                            mSurfCloudArray[i - 1 + mCloudWidth * j + mCloudWidth * mCloudHeight * k];
                    }
                    mCornerCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] =
                        laserCloudCubeCornerPointer;
                    mSurfCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] = laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }
            ++center_cubeI;
            ++mCloudCenWidth;
        }

        while (center_cubeI >= mCloudWidth - 3)
        {
            for (int j = 0; j < mCloudHeight; ++j)
            {
                for (int k = 0; k < mCloudDepth; ++k)
                {
                    int i = 0;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                        mCornerCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                        mSurfCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k];
                    for (; i < mCloudWidth - 1; i++)
                    {
                        mCornerCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] =
                            mCornerCloudArray[i + 1 + mCloudWidth * j + mCloudWidth * mCloudHeight * k];
                        mSurfCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] =
                            mSurfCloudArray[i + 1 + mCloudWidth * j + mCloudWidth * mCloudHeight * k];
                    }
                    mCornerCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] =
                        laserCloudCubeCornerPointer;
                    mSurfCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] = laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }
            --center_cubeI;
            --mCloudCenWidth;
        }

        while (center_cubeJ < 3)
        {
            for (int i = 0; i < mCloudWidth; ++i)
            {
                for (int k = 0; k < mCloudDepth; ++k)
                {
                    int j = mCloudHeight - 1;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                        mCornerCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                        mSurfCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k];
                    for (; j >= 1; j--)
                    {
                        mCornerCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] =
                            mCornerCloudArray[i + mCloudWidth * (j - 1) + mCloudWidth * mCloudHeight * k];
                        mSurfCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] =
                            mSurfCloudArray[i + mCloudWidth * (j - 1) + mCloudWidth * mCloudHeight * k];
                    }
                    mCornerCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] =
                        laserCloudCubeCornerPointer;
                    mSurfCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] = laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }
            ++center_cubeJ;
            ++mCloudCenHeight;
        }

        while (center_cubeJ >= mCloudHeight - 3)
        {
            for (int i = 0; i < mCloudWidth; ++i)
            {
                for (int k = 0; k < mCloudDepth; ++k)
                {
                    int j = 0;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                        mCornerCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                        mSurfCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k];
                    for (; j < mCloudHeight - 1; j++)
                    {
                        mCornerCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] =
                            mCornerCloudArray[i + mCloudWidth * (j + 1) + mCloudWidth * mCloudHeight * k];
                        mSurfCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] =
                            mSurfCloudArray[i + mCloudWidth * (j + 1) + mCloudWidth * mCloudHeight * k];
                    }
                    mCornerCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] =
                        laserCloudCubeCornerPointer;
                    mSurfCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] = laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }
            --center_cubeJ;
            --mCloudCenHeight;
        }

        while (center_cubeK < 3)
        {
            for (int i = 0; i < mCloudWidth; ++i)
            {
                for (int j = 0; j < mCloudHeight; ++j)
                {
                    int k = mCloudDepth - 1;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                        mCornerCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                        mSurfCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k];
                    for (; k >= 1; k--)
                    {
                        mCornerCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] =
                            mCornerCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * (k - 1)];
                        mSurfCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] =
                            mSurfCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * (k - 1)];
                    }
                    mCornerCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] =
                        laserCloudCubeCornerPointer;
                    mSurfCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] = laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }
            ++center_cubeK;
            ++mCloudCenDepth;
        }

        while (center_cubeK >= mCloudDepth - 3)
        {
            for (int i = 0; i < mCloudWidth; ++i)
            {
                for (int j = 0; j < mCloudHeight; ++j)
                {
                    int k = 0;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                        mCornerCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                        mSurfCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k];
                    for (; k < mCloudDepth - 1; k++)
                    {
                        mCornerCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] =
                            mCornerCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * (k + 1)];
                        mSurfCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] =
                            mSurfCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * (k + 1)];
                    }
                    mCornerCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] =
                        laserCloudCubeCornerPointer;
                    mSurfCloudArray[i + mCloudWidth * j + mCloudWidth * mCloudHeight * k] = laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }
            --center_cubeK;
            --mCloudCenDepth;
        }

        int valid_cloud_num = 0;
        int surround_cloud_num = 0;

        for (int i = center_cubeI - 2; i <= center_cubeI + 2; ++i)
        {
            for (int j = center_cubeJ - 2; j <= center_cubeJ + 2; ++j)
            {
                for (int k = center_cubeK - 1; k <= center_cubeK + 1; ++k)
                {
                    if (i >= 0 && i < mCloudWidth && j >= 0 && j < mCloudHeight && k >= 0 && k < mCloudDepth)
                    {
                        mCloudValidInd[valid_cloud_num] = i + mCloudWidth * j + mCloudWidth * mCloudHeight * k;
                        ++valid_cloud_num;
                        mCloudSurroundInd[surround_cloud_num] = i + mCloudWidth * j + mCloudWidth * mCloudHeight * k;
                        ++surround_cloud_num;
                    }
                }
            }
        }
        mCornerFromMapCloud->clear();
        mSurfFromMapCloud->clear();
        for (int i = 0; i < valid_cloud_num; ++i)
        {
            *mCornerFromMapCloud += *mCornerCloudArray[mCloudValidInd[i]];
            *mSurfFromMapCloud += *mSurfCloudArray[mCloudValidInd[i]];
        }
        int corner_cloud_from_map_num = mCornerFromMapCloud->points.size();
        int surf_cloud_from_map_num = mSurfFromMapCloud->points.size();

        pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
        mCornerDownSizeFilter.setInputCloud(corner_cloud);
        mCornerDownSizeFilter.filter(*laserCloudCornerStack);
        int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

        pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
        mSurfDownSizeFilter.setInputCloud(surf_cloud);
        mSurfDownSizeFilter.filter(*laserCloudSurfStack);
        int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

        // std::printf("map corner num %d  surf num %d \n", corner_cloud_from_map_num, surf_cloud_from_map_num);
        if (corner_cloud_from_map_num > 10 && surf_cloud_from_map_num > 50)
        {
            mCornerFromMapKDTree->setInputCloud(mCornerFromMapCloud);
            mSurfFromMapKDTree->setInputCloud(mSurfFromMapCloud);

            for (int iterCount = 0; iterCount < 2; ++iterCount)
            {
                ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);
                ceres::LocalParameterization* q_parameterization = new ceres::EigenQuaternionParameterization();
                ceres::Problem::Options problem_options;

                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(parameters, 4, q_parameterization);
                problem.AddParameterBlock(parameters + 4, 3);

                int corner_num = 0;

                for (int i = 0; i < laserCloudCornerStackNum; ++i)
                {
                    pointOri = laserCloudCornerStack->points[i];
                    pointAssociateToMap(&pointOri, &pointSel);
                    mCornerFromMapKDTree->nearestKSearch(pointSel, 5, mPointSearchInd, mPointSearchSqDis);

                    if (mPointSearchSqDis[4] < 1.0)
                    {
                        std::vector<Eigen::Vector3d> nearCorners;
                        Eigen::Vector3d center(0, 0, 0);
                        for (int j = 0; j < 5; ++j)
                        {
                            Eigen::Vector3d tmp(mCornerFromMapCloud->points[mPointSearchInd[j]].x,
                                                mCornerFromMapCloud->points[mPointSearchInd[j]].y,
                                                mCornerFromMapCloud->points[mPointSearchInd[j]].z);
                            center = center + tmp;
                            nearCorners.push_back(tmp);
                        }
                        center = center / 5.0;

                        Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
                        for (int j = 0; j < 5; ++j)
                        {
                            Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                            covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                        }

                        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

                        // if is indeed line feature
                        // note Eigen library sort eigenvalues in increasing order
                        Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                        Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                        if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
                        {
                            Eigen::Vector3d point_on_line = center;
                            Eigen::Vector3d point_a, point_b;
                            point_a = 0.1 * unit_direction + point_on_line;
                            point_b = -0.1 * unit_direction + point_on_line;

                            ceres::CostFunction* cost_function =
                                LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
                            problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                            ++corner_num;
                        }
                    }
                }
                int surf_num = 0;
                for (int i = 0; i < laserCloudSurfStackNum; ++i)
                {
                    pointOri = laserCloudSurfStack->points[i];
                    pointAssociateToMap(&pointOri, &pointSel);
                    mSurfFromMapKDTree->nearestKSearch(pointSel, 5, mPointSearchInd, mPointSearchSqDis);

                    Eigen::Matrix<double, 5, 3> matA0;
                    Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
                    if (mPointSearchSqDis[4] < 1.0)
                    {
                        for (int j = 0; j < 5; ++j)
                        {
                            matA0(j, 0) = mSurfFromMapCloud->points[mPointSearchInd[j]].x;
                            matA0(j, 1) = mSurfFromMapCloud->points[mPointSearchInd[j]].y;
                            matA0(j, 2) = mSurfFromMapCloud->points[mPointSearchInd[j]].z;
                        }

                        // Find the norm of plane
                        Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                        double negative_OA_dot_norm = 1 / norm.norm();
                        norm.normalize();

                        // Here n(pa, pb, pc) is unit norm of plane
                        bool planeValid = true;
                        for (int j = 0; j < 5; ++j)
                        {
                            // if OX * n > 0.2, then plane is not fit well
                            if (fabs(norm(0) * mSurfFromMapCloud->points[mPointSearchInd[j]].x +
                                     norm(1) * mSurfFromMapCloud->points[mPointSearchInd[j]].y +
                                     norm(2) * mSurfFromMapCloud->points[mPointSearchInd[j]].z + negative_OA_dot_norm) >
                                0.2)
                            {
                                planeValid = false;
                                break;
                            }
                        }
                        Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                        if (planeValid)
                        {
                            ceres::CostFunction* cost_function =
                                LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
                            problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                            surf_num++;
                        }
                    }
                }
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 4;
                options.minimizer_progress_to_stdout = false;
                options.check_gradients = false;
                options.gradient_check_relative_precision = 1e-4;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
            }
        }
        // else
        // {
        //     RCLCPP_WARN(get_logger(), "time Map corner and surf num are not enough");
        // }
        transformUpdate();

        for (int i = 0; i < laserCloudCornerStackNum; ++i)
        {
            pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);

            int cubeI = static_cast<int>((pointSel.x + 25.0) / 50.0) + mCloudCenWidth;
            int cubeJ = static_cast<int>((pointSel.y + 25.0) / 50.0) + mCloudCenHeight;
            int cubeK = static_cast<int>((pointSel.z + 25.0) / 50.0) + mCloudCenDepth;

            if (pointSel.x + 25.0 < 0) --cubeI;
            if (pointSel.y + 25.0 < 0) --cubeJ;
            if (pointSel.z + 25.0 < 0) --cubeK;

            if (cubeI >= 0 && cubeI < mCloudWidth && cubeJ >= 0 && cubeJ < mCloudHeight && cubeK >= 0 &&
                cubeK < mCloudDepth)
            {
                int cubeInd = cubeI + mCloudWidth * cubeJ + mCloudWidth * mCloudHeight * cubeK;
                mCornerCloudArray[cubeInd]->push_back(pointSel);
            }
        }

        for (int i = 0; i < laserCloudSurfStackNum; ++i)
        {
            pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

            int cubeI = static_cast<int>((pointSel.x + 25.0) / 50.0) + mCloudCenWidth;
            int cubeJ = static_cast<int>((pointSel.y + 25.0) / 50.0) + mCloudCenHeight;
            int cubeK = static_cast<int>((pointSel.z + 25.0) / 50.0) + mCloudCenDepth;

            if (pointSel.x + 25.0 < 0) --cubeI;
            if (pointSel.y + 25.0 < 0) --cubeJ;
            if (pointSel.z + 25.0 < 0) --cubeK;

            if (cubeI >= 0 && cubeI < mCloudWidth && cubeJ >= 0 && cubeJ < mCloudHeight && cubeK >= 0 &&
                cubeK < mCloudDepth)
            {
                int cubeInd = cubeI + mCloudWidth * cubeJ + mCloudWidth * mCloudHeight * cubeK;
                mSurfCloudArray[cubeInd]->push_back(pointSel);
            }
        }
        for (int i = 0; i < valid_cloud_num; ++i)
        {
            int index = mCloudValidInd[i];

            pcl::PointCloud<PointType>::Ptr temp_corner_cloud(new pcl::PointCloud<PointType>());
            mCornerDownSizeFilter.setInputCloud(mCornerCloudArray[index]);
            mCornerDownSizeFilter.filter(*temp_corner_cloud);
            mCornerCloudArray[index] = temp_corner_cloud;

            pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
            mSurfDownSizeFilter.setInputCloud(mSurfCloudArray[index]);
            mSurfDownSizeFilter.filter(*tmpSurf);
            mSurfCloudArray[index] = tmpSurf;
        }

        // Publish surround map
        mSurroundCloud->clear();
        for (int i = 0; i < surround_cloud_num; ++i)
        {
            int index = mCloudSurroundInd[i];
            *mSurroundCloud += *mCornerCloudArray[index];
            *mSurroundCloud += *mSurfCloudArray[index];
        }
        // sensor_msgs::msg::PointCloud2 surround_cloud_msg_3;
        // pcl::toROSMsg(*mSurroundCloud, surround_cloud_msg_3);
        // surround_cloud_msg_3.header.stamp = this->now();
        // surround_cloud_msg_3.header.frame_id = "camera_init";
        // mSurroundCloudPub->publish(surround_cloud_msg_3);


        pcl::PointCloud<PointType>::Ptr cloud_map = std::make_shared<pcl::PointCloud<PointType>>();
        for (int i = 0; i < 4851; ++i)
        {
            *cloud_map += *mCornerCloudArray[i];
            *cloud_map += *mSurfCloudArray[i];
        }

        // sensor_msgs::msg::PointCloud2 cloud_map_msg;
        // pcl::toROSMsg(cloud_map, cloud_map_msg);
        // cloud_map_msg.header.stamp = this->now();
        // cloud_map_msg.header.frame_id = "cloud_map";
        // mCloudMapPub->publish(cloud_map_msg);

        // int full_cloud_num = full_cloud->points.size();
        // for (int i = 0; i < full_cloud_num; ++i)
        // {
        //     pointAssociateToMap(&full_cloud->points[i], &full_cloud->points[i]);
        // }

        // sensor_msgs::msg::PointCloud2 full_cloud_msg_3;
        // pcl::toROSMsg(*full_cloud, full_cloud_msg_3);
        // full_cloud_msg_3.header.stamp = this->now();
        // full_cloud_msg_3.header.frame_id = "camera_init";
        // mFullResolutionCloudPub->publish(full_cloud_msg_3);

        // nav_msgs::msg::Odometry odom_msg;
        // odom_msg.header.frame_id = "cloud_map";
        // odom_msg.child_frame_id = "odom";
        // odom_msg.header.stamp = this->now();
        // odom_msg.pose.pose.orientation.x = q_w_curr.x();
        // odom_msg.pose.pose.orientation.y = q_w_curr.y();
        // odom_msg.pose.pose.orientation.z = q_w_curr.z();
        // odom_msg.pose.pose.orientation.w = q_w_curr.w();
        // odom_msg.pose.pose.position.x = t_w_curr.x();
        // odom_msg.pose.pose.position.y = t_w_curr.y();
        // odom_msg.pose.pose.position.z = t_w_curr.z();
        // mOdomPub->publish(odom_msg);

        // geometry_msgs::msg::PoseStamped pose_msg;
        // pose_msg.header = odom_msg.header;
        // pose_msg.pose = odom_msg.pose.pose;
        // mPath.header.stamp = odom_msg.header.stamp;
        // mPath.header.frame_id = "cloud_map";
        // mPath.poses.push_back(pose_msg);
        // mPathPub->publish(mPath);

        // geometry_msgs::msg::TransformStamped tf_msg;
        // tf_msg.header.frame_id = "cloud_map";
        // tf_msg.child_frame_id = "odom";
        // tf_msg.header.stamp = odom_msg.header.stamp;
        // tf_msg.transform.translation = tf2::toMsg(tf2::Vector3(t_w_curr(0), t_w_curr(1), t_w_curr(2)));
        // tf_msg.transform.rotation = tf2::toMsg(tf2::Quaternion(q_w_curr.x(), q_w_curr.y(), q_w_curr.z(),
        // q_w_curr.w())); mTfBroadcaster->sendTransform(tf_msg);
        boost::asio::post(mThreadPool,
                          std::bind(&Mapping::publishMsgs, this, q_w_curr, t_w_curr, mSurroundCloud, cloud_map));
    }
}