#include "Mapping.h"
#include <execution>
#include <fstream>

namespace faster_lio
{
Mapping::Mapping(const std::string& name) : rclcpp::Node(name)
{
    mPreprocessor.reset(new PointCloudPreprocess());
    mImuProcessor.reset(new ImuProcess());

    declare_parameter("Common.LidarTopic", "raw_cloud");
    declare_parameter("Common.ImuTopic", "raw_imu");
    declare_parameter("Common.TimeSync", true);

    declare_parameter("Preprocess.LidarType", 2);
    declare_parameter("Preprocess.ScanLine", 16);
    declare_parameter("Preprocess.Blind", 0.01);
    declare_parameter("Preprocess.TimeScale", 1e-3);

    declare_parameter("Mapping.AccCov", 0.1);
    declare_parameter("Mapping.GyrCov", 0.1);
    declare_parameter("Mapping.AccCovB", 0.0001);
    declare_parameter("Mapping.GyrCovB", 0.0001);
    declare_parameter("Mapping.DetRange", 300.0f);
    declare_parameter("Mapping.ExtrinsicEstimation", true);
    declare_parameter("Mapping.ExtrinsicT", std::vector<double>());
    declare_parameter("Mapping.ExtrinsicR", std::vector<double>());

    declare_parameter("Publish.Path", true);
    declare_parameter("Publish.Scan", true);
    declare_parameter("Publish.Dense", false);
    declare_parameter("Publish.BodyFrameScan", true);
    declare_parameter("Publish.EffectScan", false);
    declare_parameter("Publish.TfImuFrame", "body");
    declare_parameter("Publish.TfWorldFram", "camera_init");

    declare_parameter("SavePcd.Enable", false);
    declare_parameter("SavePcd.Interval", -1);

    declare_parameter("FilterSize.Surf", 0.5);
    declare_parameter("FilterSize.Map", 0.5);

    declare_parameter("pointFilterNum", 2);
    declare_parameter("MaxIteration", 4);
    declare_parameter("CubeSideLength", 200.0);
    declare_parameter("SavePath ", true);
    declare_parameter("EnableExtractFeature", false);

    declare_parameter("IvoxGridResolution", 0.2);
    declare_parameter("IvoxNearbyType", 18);
    declare_parameter("EstiPlaneThreashold", 0.1);

    /* get params from param server */
    int lidar_type, ivox_nearby_type;

    double filter_size_surf_min;
    common::V3D lidar_T_wrt_IMU;
    common::M3D lidar_R_wrt_IMU;

    std::string lidar_topic, imu_topic;
    get_parameter("Common.LidarTopic", lidar_topic);
    get_parameter("Common.ImuTopic", imu_topic);
    get_parameter("Common.TimeSync", mEnableSyncTime);

    get_parameter("Preprocess.LidarType", lidar_type);
    get_parameter("Preprocess.ScanLine", mPreprocessor->scanNum());
    get_parameter("Preprocess.Blind", mPreprocessor->blind());
    get_parameter("Preprocess.TimeScale", mPreprocessor->timeScale());

    double acc_cov, gyr_cov, b_acc_cov, b_gyr_cov;
    get_parameter("Mapping.AccCov", acc_cov);
    get_parameter("Mapping.GyrCov", gyr_cov);
    get_parameter("Mapping.AccCovB", b_acc_cov);
    get_parameter("Mapping.GyrCovB", b_gyr_cov);
    get_parameter("Mapping.DetRange", mDetRange);
    get_parameter("Mapping.ExtrinsicEstimation", mEnableExtrinsicEst);
    get_parameter("Mapping.ExtrinsicT", mExtrinT);
    get_parameter("Mapping.ExtrinsicR", mExtrinR);

    get_parameter("Publish.Path", mEnablePubPath);
    get_parameter("Publish.Scan", mEnablePubScan);
    get_parameter("Publish.Dense", mEnablePubDense);
    get_parameter("Publish.BodyFrameScan", mEnablePubScanBody);
    get_parameter("Publish.EffectScan", mEnablePubScanEffect);
    get_parameter("Publish.TfImuFrame", mTfImuFrame);
    get_parameter("Publish.TfWorldFram", mTfWorldFrame);

    get_parameter("SavePcd.Enable", mEnableSavePcd);
    get_parameter("SavePcd.Interval", mPcdSaveInterval);

    get_parameter("FilterSize.Surf", filter_size_surf_min);
    get_parameter("FilterSize.Map", mFilterSizeMapMin);

    get_parameter("pointFilterNum", mPreprocessor->pointFilterNum());
    get_parameter("MaxIteration", options::NUM_MAX_ITERATIONS);
    get_parameter("CubeSideLength", mCubeLen);
    get_parameter("SavePath ", mEnableSavePath);
    get_parameter("EnableExtractFeature", mPreprocessor->featureEnabled());

    get_parameter("IvoxGridResolution", mIvoxOptions.resolution_);
    get_parameter("IvoxNearbyType", ivox_nearby_type);
    get_parameter("EstiPlaneThreashold", options::ESTI_PLANE_THRESHOLD);

    switch (lidar_type)
    {
        case 1:
            mPreprocessor->setLidarType(LidarType::AVIA);
            RCLCPP_INFO(get_logger(), "Using AVIA Lidar");
            break;
        case 2:
            mPreprocessor->setLidarType(LidarType::VELO32);
            RCLCPP_INFO(get_logger(), "Using Velodyne 32 Lidar");
            break;
        case 3:
            mPreprocessor->setLidarType(LidarType::OUST64);
            RCLCPP_INFO(get_logger(), "Using OUST 64 Lidar");
        default:
            mPreprocessor->setLidarType(LidarType::VELO32);
            RCLCPP_WARN(get_logger(), "Unknown lidar_type, set it to Velodyne 32 Lidar");
            break;
    }

    switch (ivox_nearby_type)
    {
        case 0:
            mIvoxOptions.nearby_type_ = IVoxType::NearbyType::CENTER;
            break;
        case 6:
            mIvoxOptions.nearby_type_ = IVoxType::NearbyType::NEARBY6;
            break;
        case 18:
            mIvoxOptions.nearby_type_ = IVoxType::NearbyType::NEARBY18;
            break;
        case 26:
            mIvoxOptions.nearby_type_ = IVoxType::NearbyType::NEARBY26;
            break;
        default:
            RCLCPP_WARN(get_logger(), "Unknown ivox_nearby_type, use NEARBY18.");
            mIvoxOptions.nearby_type_ = IVoxType::NearbyType::NEARBY18;
            break;
    }

    mScanVoxelFilter.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    lidar_T_wrt_IMU = common::VecFromArray<double>(mExtrinT);
    lidar_R_wrt_IMU = common::MatFromArray<double>(mExtrinR);

    mImuProcessor->setExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    mImuProcessor->setGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    mImuProcessor->setAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    mImuProcessor->setGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    mImuProcessor->setAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));


    mPath.header.stamp = this->now();
    mPath.header.frame_id = "camera_init";

    /* Local map init (after loadParams) */
    mIvox = std::make_shared<IVoxType>(mIvoxOptions);

    /* Initiate ESEKF */
    std::vector<double> epsi(23, 0.001);
    mESEKF.init_dyn_share(
        get_f,
        df_dx,
        df_dw,
        [this](state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) { obsModel(s, ekfom_data); },
        options::NUM_MAX_ITERATIONS,
        epsi.data());

    mTfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    /* Initiate Subscriptions */
    mLidarSub = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic,
        rclcpp::SensorDataQoS().best_effort(),
        std::bind(&Mapping::lidarCallback, this, std::placeholders::_1));
    mImuSub = create_subscription<sensor_msgs::msg::Imu>(imu_topic,
                                                         rclcpp::SensorDataQoS().best_effort(),
                                                         std::bind(&Mapping::imuCallback, this, std::placeholders::_1));

    /* Initiate Publishers */
    mCloudWorldPub =
        create_publisher<sensor_msgs::msg::PointCloud2>("cloud_world", rclcpp::SensorDataQoS().best_effort());
    // mCloudBodyPub =
    //     create_publisher<sensor_msgs::msg::PointCloud2>("cloud_body", rclcpp::SensorDataQoS().best_effort());
    // mCloudEffectWorldPub =
    //     create_publisher<sensor_msgs::msg::PointCloud2>("cloud_effect_world", rclcpp::SensorDataQoS().best_effort());
    mAftMappedOdomPub = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS().best_effort());
    mPathPub = create_publisher<nav_msgs::msg::Path>("odom_path", rclcpp::SensorDataQoS().best_effort());

    // mThread = std::thread(&Mapping::run, this);
}


void Mapping::run()
{
    if (!syncPackages())
    {
        return;
    }

    /* IMU process, kf prediction, undistortion */
    mImuProcessor->process(mMeasures, mESEKF, mUndistortScan);
    if (mUndistortScan->empty() || (mUndistortScan == nullptr))
    {
        RCLCPP_WARN(get_logger(), "No point, skip this scan!");
        return;
    }

    /* The first scan */
    if (mFirstScanFlg)
    {
        mIvox->addPoints(mUndistortScan->points);
        mFirstLidarTime = mMeasures.lidar_bag_time;
        mFirstScanFlg = false;
        return;
    }
    mEKFInitedFlg = (mMeasures.lidar_bag_time - mFirstLidarTime) >= options::INIT_TIME;

    /* Downsample */
    mScanVoxelFilter.setInputCloud(mUndistortScan);
    mScanVoxelFilter.filter(*mDownBodyScan);

    int cur_pts = mDownBodyScan->size();
    if (cur_pts < 5)
    {
        RCLCPP_WARN_STREAM(
            get_logger(), "Too few points, skip this scan!" << mUndistortScan->size() << ", " << mDownBodyScan->size());

        return;
    }
    mDownWorldScan->resize(cur_pts);
    mNearestPoints.resize(cur_pts);
    mResiduals.resize(cur_pts, 0);
    mSurfSelectedPoints.resize(cur_pts, true);
    mPlaneCoeffs.resize(cur_pts, common::V4F::Zero());

    /* ICP and iterated Kalman filter update */
    double solve_H_time = 0;   // iterated state estimation
    /* Update the observation model, will call nn and point-to-plane residual computation */
    mESEKF.update_iterated_dyn_share_modified(options::LASER_POINT_COV, solve_H_time);
    /* Save the state */
    mStatePoint = mESEKF.get_x();
    mCurEuler = SO3ToEuler(mStatePoint.rot);
    mLidarPosition = mStatePoint.pos + mStatePoint.rot * mStatePoint.offset_T_L_I;

    mapIncremental();   // Update local map

    RCLCPP_INFO_STREAM(get_logger(),
                       "[Mapping]: In num: " << mUndistortScan->points.size() << " downsamp " << cur_pts
                                             << " Map grid num: " << mIvox->NumValidGrids()
                                             << " effect num : " << mEffectFeatNum);

    publishOdometry();
    publishFrameWorld();
    publishPath();
    // /* publish or save map pcd */
    // if (mRunOffline)
    // {
    //     if (mEnableSavePcd)
    //     {
    //         publishFrameWorld();
    //     }
    //     if (mEnableSavePath)
    //     {
    //         publishPath();
    //     }
    // }
    // else
    // {
    //     // if (pub_odom_aft_mapped_)
    //     // {
    //     //     publishOdometry();
    //     // }
    //     publishOdometry();

    //     if (mEnablePubPath || mEnableSavePath)
    //     {
    //         publishPath();
    //     }
    //     if (mEnablePubScan || mEnableSavePcd)
    //     {
    //         publishFrameWorld();
    //     }
    //     if (mEnablePubScan && mEnablePubScanBody)
    //     {
    //         publishFrameBody();
    //     }
    //     if (mEnablePubScan && mEnablePubScanEffect)
    //     {
    //         publishFrameEffectWorld();
    //     }
    // }
    mFrameNum++;   // Debug variables
}

// void Mapping::run()
// {
//     // for (;;)
//     while (!mShutdown)
//     {
//         if (!syncPackages())
//         {
//             continue;
//         }

//         /* IMU process, kf prediction, undistortion */
//         mImuProcessor->process(mMeasures, mESEKF, mUndistortScan);
//         if (mUndistortScan->empty() || (mUndistortScan == nullptr))
//         {
//             RCLCPP_WARN(get_logger(), "No point, skip this scan!");
//             continue;
//         }

//         /* The first scan */
//         if (mFirstScanFlg)
//         {
//             mIvox->addPoints(mUndistortScan->points);
//             mFirstLidarTime = mMeasures.lidar_bag_time;
//             mFirstScanFlg = false;
//             continue;
//         }
//         mEKFInitedFlg = (mMeasures.lidar_bag_time - mFirstLidarTime) >= options::INIT_TIME;

//         /* Downsample */
//         mScanVoxelFilter.setInputCloud(mUndistortScan);
//         mScanVoxelFilter.filter(*mDownBodyScan);

//         int cur_pts = mDownBodyScan->size();
//         if (cur_pts < 5)
//         {
//             RCLCPP_WARN_STREAM(get_logger(),
//                                "Too few points, skip this scan!" << mUndistortScan->size() << ", "
//                                                                  << mDownBodyScan->size());

//             continue;
//         }
//         mDownWorldScan->resize(cur_pts);
//         mNearestPoints.resize(cur_pts);
//         mResiduals.resize(cur_pts, 0);
//         mSurfSelectedPoints.resize(cur_pts, true);
//         mPlaneCoeffs.resize(cur_pts, common::V4F::Zero());

//         /* ICP and iterated Kalman filter update */
//         double solve_H_time = 0;   // iterated state estimation
//         /* Update the observation model, will call nn and point-to-plane residual computation */
//         mESEKF.update_iterated_dyn_share_modified(options::LASER_POINT_COV, solve_H_time);
//         /* Save the state */
//         mStatePoint = mESEKF.get_x();
//         mCurEuler = SO3ToEuler(mStatePoint.rot);
//         mLidarPosition = mStatePoint.pos + mStatePoint.rot * mStatePoint.offset_T_L_I;

//         mapIncremental();   // update local map

//         publishFrameWorld();
//         publishPath();
//         publishOdometry();

//         mFrameNum++;   // Debug variables
//         std::this_thread::sleep_for(std::chrono::milliseconds(1));
//     }
// }

void Mapping::lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    mBufferMutex.lock();

    mScanCount++;
    double msg_timestamp_sec = toSec(msg->header.stamp);
    if (msg_timestamp_sec < mLidarLastTimeStamp)
    {
        RCLCPP_WARN(get_logger(), "Lidar loop back, clear buffer.");
        mLidarBuffer.clear();
    }
    PointCloudType::Ptr ptr(new PointCloudType());
    mPreprocessor->process(msg, ptr);
    mLidarBuffer.push_back(ptr);
    mTimeBuffer.push_back(msg_timestamp_sec);
    mLidarLastTimeStamp = msg_timestamp_sec;

    mBufferMutex.unlock();
}

void Mapping::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
{
    ++mPublishCount;
    sensor_msgs::msg::Imu::SharedPtr imu_msg(new sensor_msgs::msg::Imu(*msg));

    if (std::fabs(mLidarWrtImuTimeDiff) > 0.1 && mEnableSyncTime)
    {
        // msg->header.stamp = ros::Time().fromSec(mLidarWrtImuTimeDiff + msg->header.stamp.sec);
        imu_msg->header.stamp = rclcpp::Time((mLidarWrtImuTimeDiff + toSec(msg->header.stamp)) * 1e9);
    }

    double timestamp = toSec(imu_msg->header.stamp);

    mBufferMutex.lock();
    if (timestamp < mImuLastTimeStamp)
    {
        RCLCPP_WARN(get_logger(), "Imu loop back, clear buffer.");
        mImuBuffer.clear();
    }

    mImuLastTimeStamp = timestamp;
    mImuBuffer.emplace_back(imu_msg);
    mBufferMutex.unlock();
}

bool Mapping::syncPackages()
{
    if (mLidarBuffer.empty() || mImuBuffer.empty())
    {
        return false;
    }

    /*** push a lidar scan ***/
    if (!mLidarPushed)
    {
        mMeasures.lidar_cloud = mLidarBuffer.front();
        mMeasures.lidar_bag_time = mTimeBuffer.front();

        if (mMeasures.lidar_cloud->points.size() <= 1)
        {
            RCLCPP_WARN(get_logger(), "Too few input point cloud!");
            mLidarEndTime = mMeasures.lidar_bag_time + mLidarMeanScanTime;
        }
        else if (mMeasures.lidar_cloud->points.back().curvature / double(1000) < 0.5 * mLidarMeanScanTime)
        {
            mLidarEndTime = mMeasures.lidar_bag_time + mLidarMeanScanTime;
        }
        else
        {
            mScanNum++;
            mLidarEndTime = mMeasures.lidar_bag_time + mMeasures.lidar_cloud->points.back().curvature / double(1000);
            mLidarMeanScanTime +=
                (mMeasures.lidar_cloud->points.back().curvature / double(1000) - mLidarMeanScanTime) / mScanNum;
        }

        mMeasures.lidar_end_time = mLidarEndTime;
        mLidarPushed = true;
    }

    if (mImuLastTimeStamp < mLidarEndTime)
    {
        return false;
    }

    /*** push imu_ data, and pop from imu_ buffer ***/
    double imu_time = toSec(mImuBuffer.front()->header.stamp);
    mMeasures.imu_buffer.clear();
    while ((!mImuBuffer.empty()) && (imu_time < mLidarEndTime))
    {
        imu_time = toSec(mImuBuffer.front()->header.stamp);
        if (imu_time > mLidarEndTime) break;
        mMeasures.imu_buffer.push_back(mImuBuffer.front());
        mImuBuffer.pop_front();
    }

    mLidarBuffer.pop_front();
    mTimeBuffer.pop_front();
    mLidarPushed = false;
    return true;
}

void Mapping::printState(const state_ikfom& s)
{
    RCLCPP_INFO_STREAM(get_logger(),
                       "state r: " << s.rot.coeffs().transpose() << ", t: " << s.pos.transpose() << ", off r: "
                                   << s.offset_R_L_I.coeffs().transpose() << ", t: " << s.offset_T_L_I.transpose());
}

void Mapping::mapIncremental()
{
    PointVector points_to_add;
    PointVector point_no_need_downsample;

    int cur_pts = mDownBodyScan->size();
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    std::vector<size_t> index(cur_pts);
    for (size_t i = 0; i < cur_pts; ++i)
    {
        index[i] = i;
    }

    std::for_each(std::execution::unseq,
                  index.begin(),
                  index.end(),
                  [&](const size_t& i)
                  {
                      /* transform to world frame */
                      pointBodyToWorld(&(mDownBodyScan->points[i]), &(mDownWorldScan->points[i]));

                      /* decide if need add to map */
                      PointType& point_world = mDownWorldScan->points[i];
                      if (!mNearestPoints[i].empty() && mEKFInitedFlg)
                      {
                          const PointVector& points_near = mNearestPoints[i];

                          Eigen::Vector3f center =
                              ((point_world.getVector3fMap() / mFilterSizeMapMin).array().floor() + 0.5) *
                              mFilterSizeMapMin;

                          Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

                          if (std::fabs(dis_2_center.x()) > 0.5 * mFilterSizeMapMin &&
                              std::fabs(dis_2_center.y()) > 0.5 * mFilterSizeMapMin &&
                              std::fabs(dis_2_center.z()) > 0.5 * mFilterSizeMapMin)
                          {
                              point_no_need_downsample.emplace_back(point_world);
                              return;
                          }

                          bool need_add = true;
                          float dist = common::calc_dist(point_world.getVector3fMap(), center);
                          if (points_near.size() >= options::NUM_MATCH_POINTS)
                          {
                              for (int readd_i = 0; readd_i < options::NUM_MATCH_POINTS; readd_i++)
                              {
                                  if (common::calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6)
                                  {
                                      need_add = false;
                                      break;
                                  }
                              }
                          }
                          if (need_add)
                          {
                              points_to_add.emplace_back(point_world);
                          }
                      }
                      else
                      {
                          points_to_add.emplace_back(point_world);
                      }
                  });

    mIvox->addPoints(points_to_add);
    mIvox->addPoints(point_no_need_downsample);
}

/**
 * @brief Lidar point cloud registration, will be called by the eskf custom observation model
 * compute point-to-plane residual here
 * @param s kf state
 * @param ekfom_data H matrix
 */
void Mapping::obsModel(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data)
{
    int cnt_pts = mDownBodyScan->size();

    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < index.size(); ++i)
    {
        index[i] = i;
    }

    auto R_wl = (s.rot * s.offset_R_L_I).cast<float>();
    auto t_wl = (s.rot * s.offset_T_L_I + s.pos).cast<float>();

    /** closest surface search and residual computation **/
    std::for_each(std::execution::par_unseq,
                  index.begin(),
                  index.end(),
                  [&](const size_t& i)
                  {
                      PointType& point_body = mDownBodyScan->points[i];
                      PointType& point_world = mDownWorldScan->points[i];

                      /* transform to world frame */
                      common::V3F p_body = point_body.getVector3fMap();
                      point_world.getVector3fMap() = R_wl * p_body + t_wl;
                      point_world.intensity = point_body.intensity;

                      auto& points_near = mNearestPoints[i];
                      if (ekfom_data.converge)
                      {
                          /** Find the closest surfaces in the map **/
                          mIvox->getClosestPoint(point_world, points_near, options::NUM_MATCH_POINTS);
                          mSurfSelectedPoints[i] = points_near.size() >= options::MIN_NUM_MATCH_POINTS;
                          if (mSurfSelectedPoints[i])
                          {
                              mSurfSelectedPoints[i] =
                                  common::esti_plane(mPlaneCoeffs[i], points_near, options::ESTI_PLANE_THRESHOLD);
                          }
                      }

                      if (mSurfSelectedPoints[i])
                      {
                          auto temp = point_world.getVector4fMap();
                          temp[3] = 1.0;
                          float pd2 = mPlaneCoeffs[i].dot(temp);

                          bool valid_corr = p_body.norm() > 81 * pd2 * pd2;
                          if (valid_corr)
                          {
                              mSurfSelectedPoints[i] = true;
                              mResiduals[i] = pd2;
                          }
                      }
                  });


    mEffectFeatNum = 0;

    mCorrPts.resize(cnt_pts);
    mCorrNorm.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++)
    {
        if (mSurfSelectedPoints[i])
        {
            mCorrNorm[mEffectFeatNum] = mPlaneCoeffs[i];
            mCorrPts[mEffectFeatNum] = mDownBodyScan->points[i].getVector4fMap();
            mCorrPts[mEffectFeatNum][3] = mResiduals[i];

            mEffectFeatNum++;
        }
    }
    mCorrPts.resize(mEffectFeatNum);
    mCorrNorm.resize(mEffectFeatNum);

    if (mEffectFeatNum < 1)
    {
        ekfom_data.valid = false;
        RCLCPP_WARN(get_logger(), "No Effective Points!");
        return;
    }

    /*** Computation of Measurement Jacobian matrix H and measurements vector ***/
    ekfom_data.h_x = Eigen::MatrixXd::Zero(mEffectFeatNum, 12);   // 23
    ekfom_data.h.resize(mEffectFeatNum);

    index.resize(mEffectFeatNum);
    const common::M3F off_R = s.offset_R_L_I.toRotationMatrix().cast<float>();
    const common::V3F off_t = s.offset_T_L_I.cast<float>();
    const common::M3F Rt = s.rot.toRotationMatrix().transpose().cast<float>();

    std::for_each(std::execution::par_unseq,
                  index.begin(),
                  index.end(),
                  [&](const size_t& i)
                  {
                      common::V3F point_this_be = mCorrPts[i].head<3>();
                      common::M3F point_be_crossmat = SKEW_SYM_MATRIX(point_this_be);
                      common::V3F point_this = off_R * point_this_be + off_t;
                      common::M3F point_crossmat = SKEW_SYM_MATRIX(point_this);

                      /*** get the normal vector of closest surface/corner ***/
                      common::V3F norm_vec = mCorrNorm[i].head<3>();

                      /*** calculate the Measurement Jacobian matrix H ***/
                      common::V3F C(Rt * norm_vec);
                      common::V3F A(point_crossmat * C);

                      if (mEnableExtrinsicEst)
                      {
                          common::V3F B(point_be_crossmat * off_R.transpose() * C);
                          ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2],
                              B[0], B[1], B[2], C[0], C[1], C[2];
                      }
                      else
                      {
                          ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2],
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                      }

                      /*** Measurement: distance to the closest surface/corner ***/
                      ekfom_data.h(i) = -mCorrPts[i][3];
                  });
}

void Mapping::publishPath()
{
    setPoseStamp(mBodyPoseMsg);
    mBodyPoseMsg.header.stamp = rclcpp::Time(mLidarEndTime * 1e9);
    mBodyPoseMsg.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    mPath.poses.push_back(mBodyPoseMsg);
    if (mRunOffline == false)
    {
        mPathPub->publish(mPath);
    }
}

void Mapping::publishOdometry()
{
    mAftMappedOdom.header.frame_id = "camera_init";
    mAftMappedOdom.child_frame_id = "body";
    mAftMappedOdom.header.stamp = rclcpp::Time(mLidarEndTime * 1e9);
    setPoseStamp(mAftMappedOdom.pose);
    mAftMappedOdomPub->publish(mAftMappedOdom);
    auto P = mESEKF.get_P();
    for (int i = 0; i < 6; i++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        mAftMappedOdom.pose.covariance[i * 6 + 0] = P(k, 3);
        mAftMappedOdom.pose.covariance[i * 6 + 1] = P(k, 4);
        mAftMappedOdom.pose.covariance[i * 6 + 2] = P(k, 5);
        mAftMappedOdom.pose.covariance[i * 6 + 3] = P(k, 0);
        mAftMappedOdom.pose.covariance[i * 6 + 4] = P(k, 1);
        mAftMappedOdom.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id = mTfWorldFrame;
    tf_msg.child_frame_id = mTfImuFrame;
    tf_msg.header.stamp = mAftMappedOdom.header.stamp;
    tf_msg.transform.translation.x = mAftMappedOdom.pose.pose.position.x;
    tf_msg.transform.translation.y = mAftMappedOdom.pose.pose.position.y;
    tf_msg.transform.translation.z = mAftMappedOdom.pose.pose.position.z;
    tf_msg.transform.rotation = mAftMappedOdom.pose.pose.orientation;

    mTfBroadcaster->sendTransform(tf_msg);
}

void Mapping::publishFrameWorld()
{
    if (!(mRunOffline == false && mEnablePubScan) && !mEnableSavePcd)
    {
        return;
    }

    PointCloudType::Ptr cloud_world;
    if (mEnablePubDense)
    {
        PointCloudType::Ptr full_res_cloud(mUndistortScan);
        int size = full_res_cloud->points.size();
        cloud_world.reset(new PointCloudType(size, 1));
        for (int i = 0; i < size; i++)
        {
            pointBodyToWorld(&full_res_cloud->points[i], &cloud_world->points[i]);
        }
    }
    else
    {
        cloud_world = mDownWorldScan;
    }

    if (mRunOffline == false && mEnablePubScan)
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_world, cloud_msg);
        cloud_msg.header.stamp = rclcpp::Time(mLidarEndTime * 1e9);
        cloud_msg.header.frame_id = "camera_init";
        mCloudWorldPub->publish(cloud_msg);
        mPublishCount -= options::PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
       2. noted that pcd save will influence the real-time performences */
    // if (mEnableSavePcd)
    // {
    //     *mPclWaitSave += *cloud_world;

    //     static int scan_wait_num = 0;
    //     scan_wait_num++;
    //     if (mPclWaitSave->size() > 0 && mPcdSaveInterval > 0 && scan_wait_num >= mPcdSaveInterval)
    //     {
    //         mPcdIndex++;
    //         std::string all_points_dir(std::string(std::string(ROOT_DIR) + "PCD/scans_") + std::to_string(mPcdIndex)
    //         +
    //                                    std::string(".pcd"));
    //         pcl::PCDWriter pcd_writer;
    //         RCLCPP_INFO_STREAM(get_logger(), "Current scan saved to /PCD/" << all_points_dir);
    //         pcd_writer.writeBinary(all_points_dir, *mPclWaitSave);
    //         mPclWaitSave->clear();
    //         scan_wait_num = 0;
    //     }
    // }
}

void Mapping::publishFrameBody()
{
    int size = mUndistortScan->points.size();
    PointCloudType::Ptr laser_cloud_imu_body(new PointCloudType(size, 1));

    for (int i = 0; i < size; i++)
    {
        pointBodyLidarToIMU(&mUndistortScan->points[i], &laser_cloud_imu_body->points[i]);
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*laser_cloud_imu_body, cloud_msg);
    cloud_msg.header.stamp = rclcpp::Time(mLidarEndTime * 1e9);
    cloud_msg.header.frame_id = "body";
    mCloudBodyPub->publish(cloud_msg);
    mPublishCount -= options::PUBFRAME_PERIOD;
}

void Mapping::publishFrameEffectWorld()
{
    int size = mCorrPts.size();
    PointCloudType::Ptr laser_cloud(new PointCloudType(size, 1));

    for (int i = 0; i < size; i++)
    {
        pointBodyToWorld(mCorrPts[i].head<3>(), &laser_cloud->points[i]);
    }
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*laser_cloud, cloud_msg);
    cloud_msg.header.stamp = rclcpp::Time(mLidarEndTime * 1e9);
    cloud_msg.header.frame_id = "camera_init";
    mCloudEffectWorldPub->publish(cloud_msg);
    mPublishCount -= options::PUBFRAME_PERIOD;
}

void Mapping::saveTrajectory(const std::string& traj_file)
{
    std::ofstream ofs;
    ofs.open(traj_file, std::ios::out);
    if (!ofs.is_open())
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to open traj_file: " << traj_file);
        return;
    }

    ofs << "#timestamp x y z q_x q_y q_z q_w" << std::endl;
    for (const auto& p : mPath.poses)
    {
        ofs << std::fixed << std::setprecision(6) << toSec(p.header.stamp) << " " << std::setprecision(15)
            << p.pose.position.x << " " << p.pose.position.y << " " << p.pose.position.z << " " << p.pose.orientation.x
            << " " << p.pose.orientation.y << " " << p.pose.orientation.z << " " << p.pose.orientation.w << std::endl;
    }

    ofs.close();
}

void Mapping::pointBodyToWorld(const PointType* pi, PointType* const po)
{
    common::V3D p_body(pi->x, pi->y, pi->z);
    common::V3D p_global(mStatePoint.rot * (mStatePoint.offset_R_L_I * p_body + mStatePoint.offset_T_L_I) +
                         mStatePoint.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void Mapping::pointBodyToWorld(const common::V3F& pi, PointType* const po)
{
    common::V3D p_body(pi.x(), pi.y(), pi.z());
    common::V3D p_global(mStatePoint.rot * (mStatePoint.offset_R_L_I * p_body + mStatePoint.offset_T_L_I) +
                         mStatePoint.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = std::fabs(po->z);
}

void Mapping::pointBodyLidarToIMU(PointType const* const pi, PointType* const po)
{
    common::V3D p_body_lidar(pi->x, pi->y, pi->z);
    common::V3D p_body_imu(mStatePoint.offset_R_L_I * p_body_lidar + mStatePoint.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void Mapping::finish()
{
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (mPclWaitSave->size() > 0 && mEnableSavePcd)
    {
        std::string file_name = std::string("scans.pcd");
        std::string all_points_dir(std::string(std::string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        RCLCPP_INFO_STREAM(get_logger(), "current scan saved to /PCD/" << file_name);
        pcd_writer.writeBinary(all_points_dir, *mPclWaitSave);
    }
    RCLCPP_INFO(get_logger(), "Finish done.");
}
}   // namespace faster_lio