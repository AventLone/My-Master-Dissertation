#include "SlamNode.h"
#include <opencv2/core.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/common/transforms.h>

namespace pcl
{
void toColorOcTree(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, octomap::ColorOcTree& tree)
{
    /*** Set x, y, z coordinates ***/
    for (const auto& point : cloud.points)
    {
        tree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
    }
    /*** Set color ***/
    for (const auto& point : cloud.points)
    {
        tree.integrateNodeColor(point.x, point.y, point.z, point.r, point.g, point.b);
    }
    tree.updateInnerOccupancy();
}
}   // namespace pcl

SlamNode::SlamNode(const std::string& name) : rclcpp::Node(name)
{
    /******************************** Configure parameters ****************************************/
    this->declare_parameter("InputTopic.RgbImage", "");
    this->declare_parameter("InputTopic.DepthImage", "");
    this->declare_parameter("OctoMap.Resolution", 0.05);
    this->declare_parameter("OctoMap.FrameId", "octomap");
    std::string rgb_img_topic, depth_img_topic, octomap_frame_id;
    double octomap_resolution;
    this->get_parameter("InputTopic.RgbImage", rgb_img_topic);
    this->get_parameter("InputTopic.DepthImage", depth_img_topic);
    this->get_parameter("OctoMap.Resolution", octomap_resolution);
    this->get_parameter("OctoMap.FrameId", octomap_frame_id);
    /**********************************************************************************************/

    /**************************************** Config RCLCPP ****************************************/
    mRgbSub = new message_filters::Subscriber<sensor_msgs::msg::Image>(this, rgb_img_topic);
    mDepthSub = new message_filters::Subscriber<sensor_msgs::msg::Image>(this, depth_img_topic);
    mSynchronizer = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *mRgbSub, *mDepthSub);

    mMutiThreadCallback = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    mSingleThreadCallback = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    mSynchronizer->registerCallback(
        std::bind(&SlamNode::rgbDepthCallback, this, std::placeholders::_1, std::placeholders::_2));

    // mPointCloudPublisher =
    //     create_publisher<sensor_msgs::msg::PointCloud2>("orb_slam3/pointcloud", rclcpp::SensorDataQoS().reliable());
    mOctomapPublisher =
        this->create_publisher<octomap_msgs::msg::Octomap>("orb_slam3/octomap", rclcpp::SensorDataQoS().reliable());

    mTfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    mTimer[0] = this->create_wall_timer(
        std::chrono::milliseconds(200), std::bind(&SlamNode::publishOctoMap, this), mMutiThreadCallback);
    mTimer[1] = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&SlamNode::publishOdom, this), mMutiThreadCallback);

    mImuSub = this->create_subscription<sensor_msgs::msg::Imu>(
        "track_cart/imu",
        rclcpp::SensorDataQoS().reliable(),
        std::bind(&SlamNode::subImuCallback, this, std::placeholders::_1));
    /**********************************************************************************************/

    std::string this_package_path = ament_index_cpp::get_package_share_directory("orb_slam3");
    std::string setting_file = this_package_path + "/config/settings/SlamConfig.yaml";

    mSlamer = std::make_unique<ORB_SLAM3::System>(this_package_path + "/config/settings/ORBvoc.txt",
                                                  this_package_path + "/config/settings/SlamConfig.yaml",
                                                  ORB_SLAM3::System::RGBD,
                                                  this_package_path + "/model/semantic_segmentation.trt");

    mOctoMap = std::make_shared<octomap::ColorOcTree>(octomap_resolution);

    Eigen::Transform<float, 3, Eigen::Affine> tmp_tran = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
    tmp_tran.rotate(Eigen::AngleAxis<float>(M_PI / 2, Eigen::Vector3f::UnitZ()));
    tmp_tran.rotate(Eigen::AngleAxis<float>(-M_PI / 2, Eigen::Vector3f::UnitY()));
    mTrviz = tmp_tran.matrix().inverse();
    tmp_tran = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
    mTodom = tmp_tran.matrix();
    mTcr = tmp_tran.inverse();

    mOctomapMsg.header.frame_id = octomap_frame_id;
    mTfMsg.child_frame_id = "odom";
    mTfMsg.header.frame_id = octomap_frame_id;

    // mPointCloudMsg.header.frame_id = "pointcloud";
}

SlamNode::~SlamNode()
{
    if (!mPointCloudMap.empty())
    {
        pcl::io::savePCDFile("/home/avent/Public/Dissertation/Host/packages/map.pcd", mPointCloudMap);
    }
    delete mRgbSub;
    delete mDepthSub;
    delete mSynchronizer;
    mSlamer->Shutdown();
}

void SlamNode::rgbDepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                                const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
{
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        // cv_ptrRGB = cv_bridge::toCvCopy(rgb_msg, "8UC3");
        cv_ptrRGB = cv_bridge::toCvShare(rgb_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        // cv_ptrD = cv_bridge::toCvCopy(depth_msg, "16UC1");32FC1
        cv_ptrD = cv_bridge::toCvShare(depth_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    if (cv_ptrRGB->image.size() == cv_ptrD->image.size())
    {
        cv::Mat rgb_img, depth_img;
        cv::cvtColor(cv_ptrRGB->image, rgb_img, cv::COLOR_BGRA2RGB);
        cv::Mat depth = cv_ptrD->image * 1000;   // Convert the unit "meter" to "millimeter"
        depth.convertTo(depth_img, CV_16UC1);
        mTcw = mSlamer->TrackRGBD(rgb_img, depth_img, cv_ptrRGB->header.stamp.sec, mImuMeas);
        // mTcw = mSlamer->TrackRGBD(rgb_img, depth_img, cv_ptrRGB->header.stamp.sec);
        mImuMeas.clear();
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "The size of rgb image and depth image are not equal.");
        std::exit(EXIT_FAILURE);
    }
}

void SlamNode::subImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg)
{
    ORB_SLAM3::IMU::Point point(imu_msg->linear_acceleration.x,
                                imu_msg->linear_acceleration.y,
                                imu_msg->linear_acceleration.z,
                                imu_msg->angular_velocity.x,
                                imu_msg->angular_velocity.y,
                                imu_msg->angular_velocity.z,
                                imu_msg->header.stamp.sec);
    mImuMeas.emplace_back(point);
}

void SlamNode::publishOctoMap()
{
    mSlamer->getPointCloudMap(mPointCloudMap);
    if (mPointCloudMap.empty()) return;

    // mOccupancyMapper->insertPointCloud(mPointCloudMap);

    // pcl::PointCloud<pcl::PointXYZRGB> temp_pointcloud;
    pcl::transformPointCloud(mPointCloudMap, mPointCloudMap, mTrviz);

    pcl::toColorOcTree(mPointCloudMap, *mOctoMap);
    octomap_msgs::fullMapToMsg(*mOctoMap, mOctomapMsg);
    // mOctomapMsg.header.stamp = this->now();

    mOctomapPublisher->publish(mOctomapMsg);

    /*** Publish Point Cloud ***/
    // pcl::toROSMsg(mPointCloudMap, mPointCloudMsg);
    // mPointCloudPublisher->publish(mPointCloudMsg);
}

void SlamNode::publishOdom()
{
    // Eigen::Affine3d T_wc;
    Eigen::Matrix4f matrix_cw = mTcw.matrix().inverse();
    Eigen::Matrix4f matrix_cw_temp = matrix_cw;
    matrix_cw(0, 3) = matrix_cw_temp(2, 3) * 2;
    matrix_cw(1, 3) = -matrix_cw_temp(0, 3) * 2;
    matrix_cw(2, 3) = -matrix_cw_temp(1, 3) * 2;
    Eigen::Matrix3f rotation = matrix_cw_temp.topLeftCorner(3, 3);
    Eigen::Vector3f euler_angles = rotation.eulerAngles(2, 1, 0);
    Eigen::Vector3f euler_angles_temp = euler_angles;
    euler_angles(0) = euler_angles_temp(2);
    euler_angles(1) = -euler_angles_temp(0);
    euler_angles(2) = -euler_angles_temp(1);

    Eigen::AngleAxisf rotation_x(euler_angles[0], Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rotation_y(euler_angles[1], Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rotation_z(euler_angles[2], Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rotation_matrix = (rotation_z * rotation_y * rotation_x).matrix();
    Eigen::Quaternion<float> quaternion_wc(rotation_matrix);
    quaternion_wc.normalize();

    // T_wc.matrix() = mTcw.matrix().inverse().cast<double>();
    // T_wc.matrix() = matrix_cw.cast<double>();
    // // T_wc.rotate(Eigen::AngleAxis<double>(M_PI / 2, Eigen::Vector3d::UnitZ()));
    // // T_wc.rotate(Eigen::AngleAxis<double>(-M_PI / 2, Eigen::Vector3d::UnitY()));

    // geometry_msgs::msg::TransformStamped tf_msg = tf2::eigenToTransform(T_wc);
    // tf_msg.header.frame_id = "octomap";
    // tf_msg.child_frame_id = "odom";

    mTfMsg.transform.translation.x = matrix_cw(0, 3);
    mTfMsg.transform.translation.y = matrix_cw(1, 3);
    mTfMsg.transform.translation.z = matrix_cw(2, 3);
    mTfMsg.transform.rotation.w = quaternion_wc.w();
    mTfMsg.transform.rotation.x = quaternion_wc.x();
    mTfMsg.transform.rotation.y = quaternion_wc.y();
    mTfMsg.transform.rotation.z = quaternion_wc.z();

    mTfBroadcaster->sendTransform(mTfMsg);
}
