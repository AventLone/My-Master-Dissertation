#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>
#include <execution>
#include <fstream>

#include "Mapping.h"
#include "utils.h"

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
    declare_parameter("Preprocess.timeScale", 1e-3);

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
    declare_parameter("CubeSideLength", 200);
    declare_parameter("SavePath ", true);
    declare_parameter("EnableExtractFeature", false);

    declare_parameter("IvoxGridResolution", 0.2);
    declare_parameter("IvoxNearbyType", 18);
    declare_parameter("EstiPlaneThreashold", 0.1);

    declare_parameter("map_file_path", true);

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
    get_parameter("Preprocess.timeScale", mPreprocessor->timeScale());

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

    if (lidar_type == 1)
    {
        mPreprocessor->setLidarType(LidarType::AVIA);
        RCLCPP_INFO(get_logger(), "Using AVIA Lidar");
    }
    else if (lidar_type == 2)
    {
        mPreprocessor->setLidarType(LidarType::VELO32);
        RCLCPP_INFO(get_logger(), "Using Velodyne 32 Lidar");
    }
    else if (lidar_type == 3)
    {
        mPreprocessor->setLidarType(LidarType::OUST64);
        RCLCPP_INFO(get_logger(), "Using OUST 64 Lidar");
    }
    else
    {
        mPreprocessor->setLidarType(LidarType::VELO32);
        RCLCPP_WARN(get_logger(), "Unknown lidar_type, set it to Velodyne 32 Lidar");
    }

    if (ivox_nearby_type == 0)
    {
        mIvoxOptions.nearby_type_ = IVoxType::NearbyType::CENTER;
    }
    else if (ivox_nearby_type == 6)
    {
        mIvoxOptions.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    }
    else if (ivox_nearby_type == 18)
    {
        mIvoxOptions.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }
    else if (ivox_nearby_type == 26)
    {
        mIvoxOptions.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Unknown ivox_nearby_type, use NEARBY18.");
        mIvoxOptions.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    // mPath.header.stamp = this->now();
    // mPath.header.frame_id = "camera_init";

    mScanVoxelFilter.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    lidar_T_wrt_IMU = common::VecFromArray<double>(mExtrinT);
    lidar_R_wrt_IMU = common::MatFromArray<double>(mExtrinR);

    mImuProcessor->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    mImuProcessor->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    mImuProcessor->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    mImuProcessor->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    mImuProcessor->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));


    mPath.header.stamp = this->now();
    mPath.header.frame_id = "camera_init";

    // local map init (after loadParams)
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
        rclcpp::SensorDataQoS().reliable(),
        std::bind(&Mapping::lidarCallback, this, std::placeholders::_1));
    mImuSub = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS().reliable(), std::bind(&Mapping::imuCallback, this, std::placeholders::_1));

    /* Initiate Publishers */
    mCloudWorldPub =
        create_publisher<sensor_msgs::msg::PointCloud2>("cloud_world", rclcpp::ParameterEventsQoS().reliable());
    mCloudBodyPub =
        create_publisher<sensor_msgs::msg::PointCloud2>("cloud_world", rclcpp::ParameterEventsQoS().reliable());
    mCloudEffectWorldPub =
        create_publisher<sensor_msgs::msg::PointCloud2>("cloud_world", rclcpp::ParameterEventsQoS().reliable());
    mAftMappedOdomPub = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::ParameterEventsQoS().reliable());
    mPathPub = create_publisher<nav_msgs::msg::Path>("odom_path", rclcpp::ParameterEventsQoS().reliable());
}

/*
bool Mapping::InitROS(ros::NodeHandle& nh)
{
    loadParams(nh);
    SubAndPubToROS(nh);

    // localmap init (after loadParams)
    mIvox = std::make_shared<IVoxType>(mIvoxOptions);

    // esekf init
    std::vector<double> epsi(23, 0.001);
    mESEKF.init_dyn_share(
        get_f,
        df_dx,
        df_dw,
        [this](state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) { obsModel(s, ekfom_data); },
        options::NUM_MAX_ITERATIONS,
        epsi.data());

    return true;
}
*/

/*
bool Mapping::InitWithoutROS(const std::string& config_yaml)
{
    LOG(INFO) << "init laser mapping from " << config_yaml;
    if (!loadParamsFromYAML(config_yaml))
    {
        return false;
    }

    // localmap init (after loadParams)
    mIvox = std::make_shared<IVoxType>(mIvoxOptions);

    // esekf init
    std::vector<double> epsi(23, 0.001);
    mESEKF.init_dyn_share(
        get_f,
        df_dx,
        df_dw,
        [this](state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) { obsModel(s, ekfom_data); },
        options::NUM_MAX_ITERATIONS,
        epsi.data());

    if (std::is_same<IVoxType, IVox<3, IVoxNodeType::PHC, pcl::PointXYZI>>::value == true)
    {
        LOG(INFO) << "using phc ivox";
    }
    else if (std::is_same<IVoxType, IVox<3, IVoxNodeType::DEFAULT, pcl::PointXYZI>>::value == true)
    {
        LOG(INFO) << "using default ivox";
    }

    return true;
}
*/

/*
bool Mapping::loadParams(ros::NodeHandle& nh)
{
    // get params from param server
    int lidar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_surf_min;
    common::V3D lidar_T_wrt_IMU;
    common::M3D lidar_R_wrt_IMU;

    nh.param<bool>("path_save_en", mEnableSavePath, true);
    nh.param<bool>("publish/path_publish_en", mEnablePubPath, true);
    nh.param<bool>("publish/scan_publish_en", mEnablePubScan, true);
    nh.param<bool>("publish/dense_publish_en", mEnablePubDense, false);
    nh.param<bool>("publish/scan_bodyframe_pub_en", mEnablePubScanBody, true);
    nh.param<bool>("publish/scan_effect_pub_en", mEnablePubScanEffect, false);
    nh.param<std::string>("publish/tf_imu_frame", mTfImuFrame, "body");
    nh.param<std::string>("publish/tf_world_frame", mTfWorldFrame, "camera_init");

    nh.param<int>("max_iteration", options::NUM_MAX_ITERATIONS, 4);
    nh.param<float>("esti_plane_threshold", options::ESTI_PLANE_THRESHOLD, 0.1);
    nh.param<std::string>("map_file_path", mMapFilePath, "");
    nh.param<bool>("common/time_sync_en", mEnableSyncTime, false);
    nh.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
    nh.param<double>("filter_size_map", mFilterSizeMapMin, 0.0);
    nh.param<double>("cube_side_length", mCubeLen, 200);
    nh.param<float>("mapping/det_range", mDetRange, 300.f);
    nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
    nh.param<double>("preprocess/blind", mPreprocessor->blind(), 0.01);
    nh.param<float>("preprocess/time_scale", mPreprocessor->timeScale(), 1e-3);
    nh.param<int>("preprocess/lidar_type", lidar_type, 1);
    nh.param<int>("preprocess/scan_line", mPreprocessor->scanNum(), 16);
    nh.param<int>("point_filter_num", mPreprocessor->pointFilterNum(), 2);
    nh.param<bool>("feature_extract_enable", mPreprocessor->featureEnabled(), false);
    nh.param<bool>("runtime_pos_log_enable", mRuntimePosLog, true);
    nh.param<bool>("mapping/extrinsic_est_en", mEnableExtrinsicEst, true);
    nh.param<bool>("pcd_save/pcd_save_en", mEnableSavePcd, false);
    nh.param<int>("pcd_save/interval", mPcdSaveInterval, -1);
    nh.param<std::vector<double>>("mapping/extrinsic_T", mExtrinT, std::vector<double>());
    nh.param<std::vector<double>>("mapping/extrinsic_R", mExtrinR, std::vector<double>());

    nh.param<float>("ivox_grid_resolution", mIvoxOptions.resolution_, 0.2);
    nh.param<int>("ivox_nearby_type", ivox_nearby_type, 18);

    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1)
    {
        mPreprocessor->setLidarType(LidarType::AVIA);
        LOG(INFO) << "Using AVIA Lidar";
    }
    else if (lidar_type == 2)
    {
        mPreprocessor->setLidarType(LidarType::VELO32);
        LOG(INFO) << "Using Velodyne 32 Lidar";
    }
    else if (lidar_type == 3)
    {
        mPreprocessor->setLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    }
    else
    {
        LOG(WARNING) << "unknown lidar_type";
        return false;
    }

    if (ivox_nearby_type == 0)
    {
        mIvoxOptions.nearby_type_ = IVoxType::NearbyType::CENTER;
    }
    else if (ivox_nearby_type == 6)
    {
        mIvoxOptions.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    }
    else if (ivox_nearby_type == 18)
    {
        mIvoxOptions.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }
    else if (ivox_nearby_type == 26)
    {
        mIvoxOptions.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    }
    else
    {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        mIvoxOptions.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    mPath.header.stamp = this->now();
    mPath.header.frame_id = "camera_init";

    mScanVoxelFilter.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    lidar_T_wrt_IMU = common::VecFromArray<double>(mExtrinT);
    lidar_R_wrt_IMU = common::MatFromArray<double>(mExtrinR);

    mImuProcessor->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    mImuProcessor->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    mImuProcessor->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    mImuProcessor->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    mImuProcessor->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    return true;
}
*/

/*
bool Mapping::loadParamsFromYAML(const std::string& yaml_file)
{
    // get params from yaml
    int lidar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_surf_min;
    common::V3D lidar_T_wrt_IMU;
    common::M3D lidar_R_wrt_IMU;

    auto yaml = YAML::LoadFile(yaml_file);
    try
    {
        mEnablePubPath = yaml["publish"]["path_publish_en"].as<bool>();
        mEnablePubScan = yaml["publish"]["scan_publish_en"].as<bool>();
        mEnablePubDense = yaml["publish"]["dense_publish_en"].as<bool>();
        mEnablePubScanBody = yaml["publish"]["scan_bodyframe_pub_en"].as<bool>();
        mEnablePubScanEffect = yaml["publish"]["scan_effect_pub_en"].as<bool>();
        mTfImuFrame = yaml["publish"]["tf_imu_frame"].as<std::string>("body");
        mTfWorldFrame = yaml["publish"]["tf_world_frame"].as<std::string>("camera_init");
        mEnableSavePath = yaml["path_save_en"].as<bool>();

        options::NUM_MAX_ITERATIONS = yaml["max_iteration"].as<int>();
        options::ESTI_PLANE_THRESHOLD = yaml["esti_plane_threshold"].as<float>();
        mEnableSyncTime = yaml["common"]["time_sync_en"].as<bool>();

        filter_size_surf_min = yaml["filter_size_surf"].as<float>();
        mFilterSizeMapMin = yaml["filter_size_map"].as<float>();
        mCubeLen = yaml["cube_side_length"].as<int>();
        mDetRange = yaml["mapping"]["det_range"].as<float>();
        gyr_cov = yaml["mapping"]["gyr_cov"].as<float>();
        acc_cov = yaml["mapping"]["acc_cov"].as<float>();
        b_gyr_cov = yaml["mapping"]["b_gyr_cov"].as<float>();
        b_acc_cov = yaml["mapping"]["b_acc_cov"].as<float>();
        mPreprocessor->blind() = yaml["preprocess"]["blind"].as<double>();
        mPreprocessor->timeScale() = yaml["preprocess"]["time_scale"].as<double>();
        lidar_type = yaml["preprocess"]["lidar_type"].as<int>();
        mPreprocessor->scanNum() = yaml["preprocess"]["scan_line"].as<int>();
        mPreprocessor->pointFilterNum() = yaml["point_filter_num"].as<int>();
        mPreprocessor->featureEnabled() = yaml["feature_extract_enable"].as<bool>();
        mEnableExtrinsicEst = yaml["mapping"]["extrinsic_est_en"].as<bool>();
        mEnableSavePcd = yaml["pcd_save"]["pcd_save_en"].as<bool>();
        mPcdSaveInterval = yaml["pcd_save"]["interval"].as<int>();
        mExtrinT = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
        mExtrinR = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();

        mIvoxOptions.resolution_ = yaml["ivox_grid_resolution"].as<float>();
        ivox_nearby_type = yaml["ivox_nearby_type"].as<int>();
    }
    catch (...)
    {
        LOG(ERROR) << "bad conversion";
        return false;
    }

    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1)
    {
        mPreprocessor->setLidarType(LidarType::AVIA);
        LOG(INFO) << "Using AVIA Lidar";
    }
    else if (lidar_type == 2)
    {
        mPreprocessor->setLidarType(LidarType::VELO32);
        LOG(INFO) << "Using Velodyne 32 Lidar";
    }
    else if (lidar_type == 3)
    {
        mPreprocessor->setLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    }
    else
    {
        LOG(WARNING) << "unknown lidar_type";
        return false;
    }

    if (ivox_nearby_type == 0)
    {
        mIvoxOptions.nearby_type_ = IVoxType::NearbyType::CENTER;
    }
    else if (ivox_nearby_type == 6)
    {
        mIvoxOptions.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    }
    else if (ivox_nearby_type == 18)
    {
        mIvoxOptions.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }
    else if (ivox_nearby_type == 26)
    {
        mIvoxOptions.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    }
    else
    {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        mIvoxOptions.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    mScanVoxelFilter.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    lidar_T_wrt_IMU = common::VecFromArray<double>(mExtrinT);
    lidar_R_wrt_IMU = common::MatFromArray<double>(mExtrinR);

    mImuProcessor->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    mImuProcessor->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    mImuProcessor->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    mImuProcessor->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    mImuProcessor->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    mRunOffline = true;
    return true;
}
*/

/*
void Mapping::SubAndPubToROS(ros::NodeHandle& nh)
{
    // ROS subscribe initialization
    std::string lidar_topic, imu_topic;
    nh.param<std::string>("common/lid_topic", lidar_topic, "/livox/lidar");
    nh.param<std::string>("common/imu_topic", imu_topic, "/livox/imu");

    if (mPreprocessor->getLidarType() == LidarType::AVIA)
    {
        sub_pcl_ = nh.subscribe<livox_ros_driver::CustomMsg>(
            lidar_topic, 200000, [this](const livox_ros_driver::CustomMsg::ConstPtr& msg) { LivoxPCLCallBack(msg); });
    }
    else
    {
        sub_pcl_ = nh.subscribe<sensor_msgs::PointCloud2>(
            lidar_topic, 200000, [this](const sensor_msgs::PointCloud2::ConstPtr& msg) { lidarCallback(msg); });
    }

    sub_imu_ = nh.subscribe<sensor_msgs::Imu>(
        imu_topic, 200000, [this](const sensor_msgs::Imu::ConstPtr& msg) { imuCallback(msg); });

    // ROS publisher init
    mPath.header.stamp = ros::Time::now();
    mPath.header.frame_id = "camera_init";

    pub_laser_cloud_world_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
    pub_laser_cloud_body_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
    pub_laser_cloud_effect_world_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_effect_world", 100000);
    pub_odom_aft_mapped_ = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    pub_path_ = nh.advertise<nav_msgs::Path>("/path", 100000);
}
*/

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

    /// the first scan
    if (mFirstScanFlg)
    {
        mIvox->AddPoints(mUndistortScan->points);
        mFirstLidarTime = mMeasures.lidar_bag_time;
        mFirstScanFlg = false;
        return;
    }
    mEKFInitedFlg = (mMeasures.lidar_bag_time - mFirstLidarTime) >= options::INIT_TIME;

    /// downsample
    Timer::evaluate(
        [&, this]()
        {
            mScanVoxelFilter.setInputCloud(mUndistortScan);
            mScanVoxelFilter.filter(*mDownBodyScan);
        },
        "Downsample PointCloud");

    int cur_pts = mDownBodyScan->size();
    if (cur_pts < 5)
    {
        LOG(WARNING) << "Too few points, skip this scan!" << mUndistortScan->size() << ", " << mDownBodyScan->size();
        return;
    }
    mDownWorldScan->resize(cur_pts);
    mNearestPoints.resize(cur_pts);
    mResiduals.resize(cur_pts, 0);
    mSurfSelectedPoints.resize(cur_pts, true);
    mPlaneCoeffs.resize(cur_pts, common::V4F::Zero());

    // ICP and iterated Kalman filter update
    Timer::evaluate(
        [&, this]()
        {
            // iterated state estimation
            double solve_H_time = 0;
            // update the observation model, will call nn and point-to-plane residual computation
            mESEKF.update_iterated_dyn_share_modified(options::LASER_POINT_COV, solve_H_time);
            // save the state
            mStatePoint = mESEKF.get_x();
            mCurEuler = SO3ToEuler(mStatePoint.rot);
            mLidarPosition = mStatePoint.pos + mStatePoint.rot * mStatePoint.offset_T_L_I;
        },
        "IEKF Solve and Update");

    // update local map
    Timer::evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping");

    LOG(INFO) << "[ mapping ]: In num: " << mUndistortScan->points.size() << " downsamp " << cur_pts
              << " Map grid num: " << mIvox->NumValidGrids() << " effect num : " << mEffectFeatNum;

    // publish or save map pcd
    if (mRunOffline)
    {
        if (mEnableSavePcd)
        {
            publishFrameWorld();
        }
        if (mEnableSavePath)
        {
            publishPath(pub_path_);
        }
    }
    else
    {
        if (pub_odom_aft_mapped_)
        {
            publishOdometry(pub_odom_aft_mapped_);
        }
        if (mEnablePubPath || mEnableSavePath)
        {
            publishPath(pub_path_);
        }
        if (mEnablePubScan || mEnableSavePcd)
        {
            publishFrameWorld();
        }
        if (mEnablePubScan && mEnablePubScanBody)
        {
            publishFrameBody(pub_laser_cloud_body_);
        }
        if (mEnablePubScan && mEnablePubScanEffect)
        {
            publishFrameEffectWorld(pub_laser_cloud_effect_world_);
        }
    }

    // Debug variables
    mFrameNum++;
}

void Mapping::lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    mBufferMutex.lock();
    Timer::evaluate(
        [&, this]()
        {
            mScanCount++;
            if (msg->header.stamp.sec < mLidarLastTimeStamp)
            {
                RCLCPP_WARN(get_logger(), "Lidar loop back, clear buffer.");
                mLidarBuffer.clear();
            }

            PointCloudType::Ptr ptr(new PointCloudType());
            mPreprocessor->process(msg, ptr);
            mLidarBuffer.push_back(ptr);
            mTimeBuffer.push_back(msg->header.stamp.sec);
            mLidarLastTimeStamp = msg->header.stamp.sec;
        },
        "Preprocess (Standard)");
    mBufferMutex.unlock();
}

void Mapping::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
{
    ++mPublishCount;
    sensor_msgs::msg::Imu::SharedPtr imu_msg(new sensor_msgs::msg::Imu(*msg));

    if (std::fabs(mLidarWrtImuTimeDiff) > 0.1 && mEnableSyncTime)
    {
        // msg->header.stamp = ros::Time().fromSec(mLidarWrtImuTimeDiff + msg->header.stamp.sec);
        imu_msg->header.stamp = rclcpp::Time((mLidarWrtImuTimeDiff + msg->header.stamp.sec) * 1e9);
    }

    double timestamp = imu_msg->header.stamp.sec;

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
    double imu_time = mImuBuffer.front()->header.stamp.sec;
    mMeasures.imu_buffer.clear();
    while ((!mImuBuffer.empty()) && (imu_time < mLidarEndTime))
    {
        imu_time = mImuBuffer.front()->header.stamp.sec;
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
    // LOG(INFO) << "state r: " << s.rot.coeffs().transpose() << ", t: " << s.pos.transpose()
    //           << ", off r: " << s.offset_R_L_I.coeffs().transpose() << ", t: " << s.offset_T_L_I.transpose();
    RCLCPP_INFO_STREAM(get_logger(),
                       "state r: " << s.rot.coeffs().transpose() << ", t: " << s.pos.transpose() << ", off r: "
                                   << s.offset_R_L_I.coeffs().transpose() << ", t: " << s.offset_T_L_I.transpose());
}

void Mapping::MapIncremental()
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
                      PointBodyToWorld(&(mDownBodyScan->points[i]), &(mDownWorldScan->points[i]));

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

    Timer::evaluate(
        [&, this]()
        {
            mIvox->AddPoints(points_to_add);
            mIvox->AddPoints(point_no_need_downsample);
        },
        "    IVox Add Points");
}

/**
 * Lidar point cloud registration
 * will be called by the eskf custom observation model
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

    Timer::evaluate(
        [&, this]()
        {
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
                                  mIvox->GetClosestPoint(point_world, points_near, options::NUM_MATCH_POINTS);
                                  mSurfSelectedPoints[i] = points_near.size() >= options::MIN_NUM_MATCH_POINTS;
                                  if (mSurfSelectedPoints[i])
                                  {
                                      mSurfSelectedPoints[i] = common::esti_plane(
                                          mPlaneCoeffs[i], points_near, options::ESTI_PLANE_THRESHOLD);
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
        },
        "    obsModel (Lidar Match)");

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
        LOG(WARNING) << "No Effective Points!";
        return;
    }

    Timer::evaluate(
        [&, this]()
        {
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
                                  ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0],
                                      A[1], A[2], B[0], B[1], B[2], C[0], C[1], C[2];
                              }
                              else
                              {
                                  ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0],
                                      A[1], A[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                              }

                              /*** Measurement: distance to the closest surface/corner ***/
                              ekfom_data.h(i) = -mCorrPts[i][3];
                          });
        },
        "    obsModel (IEKF Build Jacobian)");
}

void Mapping::publishPath()
{
    setPoseStamp(mBodyPoseMsg);
    // mBodyPoseMsg.header.stamp = ros::Time().fromSec(mLidarEndTime);
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
    // mAftMappedOdom.header.stamp = ros::Time().fromSec(mLidarEndTime);   // ros::Time().fromSec(mLidarEndTime);
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
            PointBodyToWorld(&full_res_cloud->points[i], &cloud_world->points[i]);
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
        // cloud_msg.header.stamp = ros::Time().fromSec(mLidarEndTime);
        cloud_msg.header.stamp = rclcpp::Time(mLidarEndTime * 1e9);
        cloud_msg.header.frame_id = "camera_init";
        pub_laser_cloud_world_.publish(cloud_msg);
        mPublishCount -= options::PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (mEnableSavePcd)
    {
        *mPclWaitSave += *cloud_world;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (mPclWaitSave->size() > 0 && mPcdSaveInterval > 0 && scan_wait_num >= mPcdSaveInterval)
        {
            mPcdIndex++;
            std::string all_points_dir(std::string(std::string(ROOT_DIR) + "PCD/scans_") + std::to_string(mPcdIndex) +
                                       std::string(".pcd"));
            pcl::PCDWriter pcd_writer;
            // LOG(INFO) << "current scan saved to /PCD/" << all_points_dir;
            pcd_writer.writeBinary(all_points_dir, *mPclWaitSave);
            mPclWaitSave->clear();
            scan_wait_num = 0;
        }
    }
}

void Mapping::publishFrameBody()
{
    int size = mUndistortScan->points.size();
    PointCloudType::Ptr laser_cloud_imu_body(new PointCloudType(size, 1));

    for (int i = 0; i < size; i++)
    {
        PointBodyLidarToIMU(&mUndistortScan->points[i], &laser_cloud_imu_body->points[i]);
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*laser_cloud_imu_body, cloud_msg);
    // cloud_msg.header.stamp = ros::Time().fromSec(mLidarEndTime);
    cloud_msg.header.stamp = rclcpp::Time(mLidarEndTime * 1e9);
    cloud_msg.header.frame_id = "body";
    pub_laser_cloud_body.publish(cloud_msg);
    mPublishCount -= options::PUBFRAME_PERIOD;
}

void Mapping::publishFrameEffectWorld()
{
    int size = mCorrPts.size();
    PointCloudType::Ptr laser_cloud(new PointCloudType(size, 1));

    for (int i = 0; i < size; i++)
    {
        PointBodyToWorld(mCorrPts[i].head<3>(), &laser_cloud->points[i]);
    }
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*laser_cloud, cloud_msg);
    cloud_msg.header.stamp = rclcpp::Time(mLidarEndTime * 1e9);
    cloud_msg.header.frame_id = "camera_init";
    pub_laser_cloud_effect_world.publish(cloud_msg);
    mPublishCount -= options::PUBFRAME_PERIOD;
}

void Mapping::saveTrajectory(const std::string& traj_file)
{
    std::ofstream ofs;
    ofs.open(traj_file, std::ios::out);
    if (!ofs.is_open())
    {
        LOG(ERROR) << "Failed to open traj_file: " << traj_file;
        return;
    }

    ofs << "#timestamp x y z q_x q_y q_z q_w" << std::endl;
    for (const auto& p : mPath.poses)
    {
        ofs << std::fixed << std::setprecision(6) << p.header.stamp.toSec() << " " << std::setprecision(15)
            << p.pose.position.x << " " << p.pose.position.y << " " << p.pose.position.z << " " << p.pose.orientation.x
            << " " << p.pose.orientation.y << " " << p.pose.orientation.z << " " << p.pose.orientation.w << std::endl;
    }

    ofs.close();
}

///////////////////////////  private method /////////////////////////////////////////////////////////////////////
template<typename T>
void Mapping::setPoseStamp(T& out)
{
    out.pose.position.x = mStatePoint.pos(0);
    out.pose.position.y = mStatePoint.pos(1);
    out.pose.position.z = mStatePoint.pos(2);
    out.pose.orientation.x = mStatePoint.rot.coeffs()[0];
    out.pose.orientation.y = mStatePoint.rot.coeffs()[1];
    out.pose.orientation.z = mStatePoint.rot.coeffs()[2];
    out.pose.orientation.w = mStatePoint.rot.coeffs()[3];
}

void Mapping::PointBodyToWorld(const PointType* pi, PointType* const po)
{
    common::V3D p_body(pi->x, pi->y, pi->z);
    common::V3D p_global(mStatePoint.rot * (mStatePoint.offset_R_L_I * p_body + mStatePoint.offset_T_L_I) +
                         mStatePoint.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void Mapping::PointBodyToWorld(const common::V3F& pi, PointType* const po)
{
    common::V3D p_body(pi.x(), pi.y(), pi.z());
    common::V3D p_global(mStatePoint.rot * (mStatePoint.offset_R_L_I * p_body + mStatePoint.offset_T_L_I) +
                         mStatePoint.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = std::abs(po->z);
}

void Mapping::PointBodyLidarToIMU(PointType const* const pi, PointType* const po)
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
        LOG(INFO) << "current scan saved to /PCD/" << file_name;
        pcd_writer.writeBinary(all_points_dir, *mPclWaitSave);
    }

    LOG(INFO) << "finish done";
}
}   // namespace faster_lio