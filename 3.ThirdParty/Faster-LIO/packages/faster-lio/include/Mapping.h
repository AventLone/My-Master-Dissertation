#ifndef FASTER_LIO_LASER_MAPPING_H
#define FASTER_LIO_LASER_MAPPING_H

// #include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/msg/path.hpp>
#include <pcl/filters/voxel_grid.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <condition_variable>
#include <thread>

#include "imu_processing.hpp"
#include "ivox3d/ivox3d.h"
#include "options.h"
#include "pointcloud_preprocess.h"

namespace faster_lio
{
class Mapping : public rclcpp::Node
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

#ifdef IVOX_NODE_TYPE_PHC
    using IVoxType = IVox<3, IVoxNodeType::PHC, PointType>;
#else
    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;
#endif

    Mapping(const std::string& name);
    // ~Mapping()
    // {
    //     mDownBodyScan = nullptr;
    //     mUndistortScan = nullptr;
    //     mDownWorldScan = nullptr;
    //     LOG(INFO) << "laser mapping deconstruct";
    // }

    /// init with ros
    // bool InitROS(ros::NodeHandle& nh);

    /// init without ros
    // bool InitWithoutROS(const std::string& config_yaml);

    void run();

    /* callbacks of lidar and imu */
    void lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    // void LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr& msg);
    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);

    // sync lidar with imu
    bool syncPackages();

    /* interface of mtk, customized observation model */
    void obsModel(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data);

    /* debug save / show */
    void publishPath();
    void publishOdometry();
    void publishFrameWorld();
    void publishFrameBody();
    void publishFrameEffectWorld();
    void saveTrajectory(const std::string& traj_file);

    void finish();

private:
    /* Modules */
    IVoxType::Options mIvoxOptions;
    std::shared_ptr<IVoxType> mIvox = nullptr;                       // local map in ivox
    std::shared_ptr<PointCloudPreprocess> mPreprocessor = nullptr;   // point cloud preprocess
    std::shared_ptr<ImuProcess> mImuProcessor = nullptr;             // imu process

    /* Local map related */
    float mDetRange = 300.0f;
    double mCubeLen = 0;
    double mFilterSizeMapMin = 0;
    bool mLocalMapInited = false;

    /* Params */
    std::vector<double> mExtrinT{3, 0.0};   // lidar-imu translation
    std::vector<double> mExtrinR{9, 0.0};   // lidar-imu rotation
    std::string mMapFilePath;

    /* Point clouds data */
    CloudPtr mUndistortScan{new PointCloudType()};   // scan after undistortion
    CloudPtr mDownBodyScan{new PointCloudType()};    // downsampled scan in body
    CloudPtr mDownWorldScan{new PointCloudType()};   // downsampled scan in world
    std::vector<PointVector> mNearestPoints;         // nearest points of current scan
    common::VV4F mCorrPts;                           // inlier pts
    common::VV4F mCorrNorm;                          // inlier plane norms
    pcl::VoxelGrid<PointType> mScanVoxelFilter;      // voxel filter for current scan
    std::vector<float> mResiduals;                   // point-to-plane residuals
    std::vector<bool> mSurfSelectedPoints;           // selected points
    common::VV4F mPlaneCoeffs;                       // plane coeffs

    /* Subscription */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mLidarSub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr mImuSub;

    /* Publishers */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mCloudWorldPub, mCloudBodyPub, mCloudEffectWorldPub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mAftMappedOdomPub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mPathPub;

    std::unique_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;


    std::string mTfImuFrame;
    std::string mTfWorldFrame;

    std::mutex mBufferMutex;
    std::deque<double> mTimeBuffer;
    std::deque<PointCloudType::Ptr> mLidarBuffer;
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> mImuBuffer;
    nav_msgs::msg::Odometry mAftMappedOdom;

    /* Options */
    bool mEnableSyncTime = false;
    double mLidarWrtImuTimeDiff = 0.0;
    double mLidarLastTimeStamp = 0;
    double mLidarEndTime = 0;
    double mImuLastTimeStamp = -1.0;
    double mFirstLidarTime = 0.0;
    bool mLidarPushed = false;

    /* Statistics and flags */
    int mScanCount = 0;
    int mPublishCount = 0;
    bool mFirstScanFlg = true;
    bool mEKFInitedFlg = false;
    int mPcdIndex = 0;
    double mLidarMeanScanTime = 0.0;
    int mScanNum = 0;
    bool mTimeDiffSetFlg = false;
    int mEffectFeatNum = 0, mFrameNum = 0;

    /* EKF inputs and output */
    common::MeasureGroup mMeasures;                        // sync IMU and lidar scan
    esekfom::esekf<state_ikfom, 12, input_ikfom> mESEKF;   // ES-EKF: Error State Extended Kalman Filter
    state_ikfom mStatePoint;                               // EKF current state
    vect3 mLidarPosition;                                  // lidar position after eskf update
    Eigen::Vector3d mCurEuler = Eigen::Vector3d::Zero();   // rotation in euler angles
    bool mEnableExtrinsicEst = true;

    /* Debug show / save */
    bool mRunOffline = false;
    bool mEnablePubPath = true;
    bool mEnablePubScan = false;
    bool mEnablePubDense = false;
    bool mEnablePubScanBody = false;
    bool mEnablePubScanEffect = false;
    bool mEnableSavePcd = false;
    bool mRuntimePosLog = true;
    int mPcdSaveInterval = -1;
    bool mEnableSavePath = false;
    std::string mDataSet;

    PointCloudType::Ptr mPclWaitSave{new PointCloudType()};   // debug save
    nav_msgs::msg::Path mPath;
    geometry_msgs::msg::PoseStamped mBodyPoseMsg;

private:
    template<typename T>
    void setPoseStamp(T& out);

    void PointBodyToWorld(PointType const* pi, PointType* const po);
    void PointBodyToWorld(const common::V3F& pi, PointType* const po);
    void PointBodyLidarToIMU(PointType const* const pi, PointType* const po);

    void MapIncremental();

    void SubAndPubToROS();

    bool loadParams();
    bool loadParamsFromYAML(const std::string& yaml);

    void printState(const state_ikfom& s);
};

}   // namespace faster_lio

#endif   // FASTER_LIO_LASER_MAPPING_H