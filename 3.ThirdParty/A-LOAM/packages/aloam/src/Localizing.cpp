#include "Localizing.h"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

Localizing::Localizing()
{
    mThread = std::thread(&Localizing::process, this);
}

void Localizing::insertCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& full_cloud,
                             const pcl::PointCloud<pcl::PointXYZI>::Ptr& sharp_cloud,
                             const pcl::PointCloud<pcl::PointXYZI>::Ptr& less_sharp_cloud,
                             const pcl::PointCloud<pcl::PointXYZI>::Ptr& flat_cloud,
                             const pcl::PointCloud<pcl::PointXYZI>::Ptr& less_flat_cloud)
{
    std::unique_lock<std::mutex> lock(mBufferMutex);
    mFullCloudBuffer.emplace(full_cloud);
    mSharpCornerCloudBuffer.emplace(sharp_cloud);
    mLessSharpCornerCloudBuffer.emplace(less_sharp_cloud);
    mFlatSurfCloudBuffer.emplace(flat_cloud);
    mLessFlatSurfCloudBuffer.emplace(less_flat_cloud);
    if (mFullCloudBuffer.size() > 30)
    {
        mFullCloudBuffer.pop();
        mSharpCornerCloudBuffer.pop();
        mLessSharpCornerCloudBuffer.pop();
        mFlatSurfCloudBuffer.pop();
        mLessFlatSurfCloudBuffer.pop();
    }
    mLocalizeCondition.notify_one();
}

void Localizing::transformToStart(PointType const* const pi, PointType* const po) const
{
    // interpolation ratio
    double s;
    if (DISTORTION)
    {
        s = (pi->intensity - int(pi->intensity)) / mScanPeriod;
    }
    else
    {
        s = 1.0;
    }
    // s = 1;
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
    Eigen::Vector3d t_point_last = s * t_last_curr;
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;

    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}

void Localizing::process()
{
    for (;;)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr full_cloud, sharp_corner_cloud, less_sharp_corner_cloud, flat_surf_cloud,
            less_flat_surf_cloud;
        {
            std::unique_lock<std::mutex> lock(mBufferMutex);
            mLocalizeCondition.wait(lock, [this] { return mFullCloudBuffer.size() > 0 || mShutDown; });
            if (mShutDown) break;

            full_cloud = mFullCloudBuffer.front();
            mFullCloudBuffer.pop();
            sharp_corner_cloud = mSharpCornerCloudBuffer.front();
            mSharpCornerCloudBuffer.pop();
            less_sharp_corner_cloud = mLessSharpCornerCloudBuffer.front();
            mLessSharpCornerCloudBuffer.pop();
            flat_surf_cloud = mFlatSurfCloudBuffer.front();
            mFlatSurfCloudBuffer.pop();
            less_flat_surf_cloud = mLessFlatSurfCloudBuffer.front();
            mLessFlatSurfCloudBuffer.pop();
        }

        // initializing
        if (!mSystemInited)
        {
            mSystemInited = true;
            std::printf("Initialization finished \n");
        }
        else
        {
            int sharp_corner_cloud_num = sharp_corner_cloud->points.size();
            int flat_sur_cloud_num = flat_surf_cloud->points.size();

            // TicToc t_opt;
            for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
            {
                mCornerCorrespondence = 0;
                mPlaneCorrespondence = 0;

                // ceres::LossFunction *loss_function = NULL;
                ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);
                ceres::LocalParameterization* q_parameterization = new ceres::EigenQuaternionParameterization();
                ceres::Problem::Options problem_options;

                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(para_q, 4, q_parameterization);
                problem.AddParameterBlock(para_t, 3);

                pcl::PointXYZI pointSel;
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;

                // find correspondence for corner features
                for (int i = 0; i < sharp_corner_cloud_num; ++i)
                {
                    transformToStart(&(sharp_corner_cloud->points[i]), &pointSel);
                    mCornerKDTree->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                    int closest_point_index = -1, min_point_index2 = -1;
                    if (pointSearchSqDis[0] < mDistanceSQthreshold)
                    {
                        closest_point_index = pointSearchInd[0];
                        int closestPointScanID = int(mCornerCloud->points[closest_point_index].intensity);

                        double minPointSqDis2 = mDistanceSQthreshold;

                        // search in the direction of increasing scan line
                        for (int j = closest_point_index + 1; j < (int)mCornerCloud->points.size(); ++j)
                        {
                            // if in the same scan line, continue
                            if (int(mCornerCloud->points[j].intensity) <= closestPointScanID) continue;

                            // if not in nearby scans, end the loop
                            if (int(mCornerCloud->points[j].intensity) > (closestPointScanID + mNearbyScan)) break;

                            double pointSqDis =
                                (mCornerCloud->points[j].x - pointSel.x) * (mCornerCloud->points[j].x - pointSel.x) +
                                (mCornerCloud->points[j].y - pointSel.y) * (mCornerCloud->points[j].y - pointSel.y) +
                                (mCornerCloud->points[j].z - pointSel.z) * (mCornerCloud->points[j].z - pointSel.z);

                            if (pointSqDis < minPointSqDis2)
                            {
                                // find nearer point
                                minPointSqDis2 = pointSqDis;
                                min_point_index2 = j;
                            }
                        }

                        // search in the direction of decreasing scan line
                        for (int j = closest_point_index - 1; j >= 0; --j)
                        {
                            // if in the same scan line, continue
                            if (int(mCornerCloud->points[j].intensity) >= closestPointScanID) continue;

                            // if not in nearby scans, end the loop
                            if (int(mCornerCloud->points[j].intensity) < (closestPointScanID - mNearbyScan)) break;

                            double pointSqDis =
                                (mCornerCloud->points[j].x - pointSel.x) * (mCornerCloud->points[j].x - pointSel.x) +
                                (mCornerCloud->points[j].y - pointSel.y) * (mCornerCloud->points[j].y - pointSel.y) +
                                (mCornerCloud->points[j].z - pointSel.z) * (mCornerCloud->points[j].z - pointSel.z);

                            if (pointSqDis < minPointSqDis2)
                            {
                                // find nearer point
                                minPointSqDis2 = pointSqDis;
                                min_point_index2 = j;
                            }
                        }
                    }
                    if (min_point_index2 >= 0)   // both closest_point_index and min_point_index2 is valid
                    {
                        Eigen::Vector3d curr_point(sharp_corner_cloud->points[i].x,
                                                   sharp_corner_cloud->points[i].y,
                                                   sharp_corner_cloud->points[i].z);
                        Eigen::Vector3d last_point_a(mCornerCloud->points[closest_point_index].x,
                                                     mCornerCloud->points[closest_point_index].y,
                                                     mCornerCloud->points[closest_point_index].z);
                        Eigen::Vector3d last_point_b(mCornerCloud->points[min_point_index2].x,
                                                     mCornerCloud->points[min_point_index2].y,
                                                     mCornerCloud->points[min_point_index2].z);

                        double s;
                        if (DISTORTION)
                            s = (sharp_corner_cloud->points[i].intensity -
                                 int(sharp_corner_cloud->points[i].intensity)) /
                                mScanPeriod;
                        else
                            s = 1.0;
                        ceres::CostFunction* cost_function =
                            LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                        problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                        ++mCornerCorrespondence;
                    }
                }

                // find correspondence for plane features
                for (int i = 0; i < flat_sur_cloud_num; ++i)
                {
                    transformToStart(&(flat_surf_cloud->points[i]), &pointSel);
                    mSurfKDTree->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                    int closest_point_index = -1, min_point_index2 = -1, minPointInd3 = -1;
                    if (pointSearchSqDis[0] < mDistanceSQthreshold)
                    {
                        closest_point_index = pointSearchInd[0];

                        // get closest point's scan ID
                        int closestPointScanID = int(mSurfCloud->points[closest_point_index].intensity);
                        double minPointSqDis2 = mDistanceSQthreshold, minPointSqDis3 = mDistanceSQthreshold;

                        // search in the direction of increasing scan line
                        for (int j = closest_point_index + 1; j < (int)mSurfCloud->points.size(); ++j)
                        {
                            // if not in nearby scans, end the loop
                            if (int(mSurfCloud->points[j].intensity) > (closestPointScanID + mNearbyScan)) break;

                            double pointSqDis =
                                (mSurfCloud->points[j].x - pointSel.x) * (mSurfCloud->points[j].x - pointSel.x) +
                                (mSurfCloud->points[j].y - pointSel.y) * (mSurfCloud->points[j].y - pointSel.y) +
                                (mSurfCloud->points[j].z - pointSel.z) * (mSurfCloud->points[j].z - pointSel.z);

                            // if in the same or lower scan line
                            if (int(mSurfCloud->points[j].intensity) <= closestPointScanID &&
                                pointSqDis < minPointSqDis2)
                            {
                                minPointSqDis2 = pointSqDis;
                                min_point_index2 = j;
                            }
                            // if in the higher scan line
                            else if (int(mSurfCloud->points[j].intensity) > closestPointScanID &&
                                     pointSqDis < minPointSqDis3)
                            {
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }

                        // search in the direction of decreasing scan line
                        for (int j = closest_point_index - 1; j >= 0; --j)
                        {
                            // if not in nearby scans, end the loop
                            if (int(mSurfCloud->points[j].intensity) < (closestPointScanID - mNearbyScan)) break;

                            double pointSqDis =
                                (mSurfCloud->points[j].x - pointSel.x) * (mSurfCloud->points[j].x - pointSel.x) +
                                (mSurfCloud->points[j].y - pointSel.y) * (mSurfCloud->points[j].y - pointSel.y) +
                                (mSurfCloud->points[j].z - pointSel.z) * (mSurfCloud->points[j].z - pointSel.z);

                            // if in the same or higher scan line
                            if (int(mSurfCloud->points[j].intensity) >= closestPointScanID &&
                                pointSqDis < minPointSqDis2)
                            {
                                minPointSqDis2 = pointSqDis;
                                min_point_index2 = j;
                            }
                            else if (int(mSurfCloud->points[j].intensity) < closestPointScanID &&
                                     pointSqDis < minPointSqDis3)
                            {
                                // find nearer point
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }

                        if (min_point_index2 >= 0 && minPointInd3 >= 0)
                        {
                            Eigen::Vector3d curr_point(flat_surf_cloud->points[i].x,
                                                       flat_surf_cloud->points[i].y,
                                                       flat_surf_cloud->points[i].z);
                            Eigen::Vector3d last_point_a(mSurfCloud->points[closest_point_index].x,
                                                         mSurfCloud->points[closest_point_index].y,
                                                         mSurfCloud->points[closest_point_index].z);
                            Eigen::Vector3d last_point_b(mSurfCloud->points[min_point_index2].x,
                                                         mSurfCloud->points[min_point_index2].y,
                                                         mSurfCloud->points[min_point_index2].z);
                            Eigen::Vector3d last_point_c(mSurfCloud->points[minPointInd3].x,
                                                         mSurfCloud->points[minPointInd3].y,
                                                         mSurfCloud->points[minPointInd3].z);

                            double s;
                            if (DISTORTION)
                            {
                                s = (flat_surf_cloud->points[i].intensity - int(flat_surf_cloud->points[i].intensity)) /
                                    mScanPeriod;
                            }
                            else
                            {
                                s = 1.0;
                            }
                            ceres::CostFunction* cost_function =
                                LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                            ++mPlaneCorrespondence;
                        }
                    }
                }
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 4;
                options.minimizer_progress_to_stdout = false;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
            }
            t_w_curr = t_w_curr + q_w_curr * t_last_curr;
            q_w_curr = q_w_curr * q_last_curr;
        }

        pcl::PointCloud<PointType>::Ptr temp_cloud = less_sharp_corner_cloud;
        less_sharp_corner_cloud = mCornerCloud;
        mCornerCloud = temp_cloud;

        temp_cloud = less_flat_surf_cloud;
        less_flat_surf_cloud = mSurfCloud;
        mSurfCloud = temp_cloud;

        mCornerCloudNum = mCornerCloud->points.size();
        mSurfCloudNum = mSurfCloud->points.size();

        mCornerKDTree->setInputCloud(mCornerCloud);
        mSurfKDTree->setInputCloud(mSurfCloud);

        if (mMapper == nullptr)
        {
            std::cout << "Mapper is nullptr!" << std::endl;
        }
        else
        {
            mMapper->insertCloudAndOdom(full_cloud, mCornerCloud, mSurfCloud, q_w_curr, t_w_curr);
        }
    }
}