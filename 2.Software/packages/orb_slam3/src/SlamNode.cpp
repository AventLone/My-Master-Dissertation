#include "SlamNode.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

static inline double getSec(const builtin_interfaces::msg::Time& time_stamp)
{
    return static_cast<double>(time_stamp.sec) + static_cast<double>(time_stamp.nanosec) * 1e-9;
}

SlamNode::SlamNode(const std::string& name) : rclcpp::Node(name)
{
    /* Configure parameters */
    declare_parameter("InputTopic.RgbImage", "BD_Roamer/rgb_camera/image_color");
    declare_parameter("InputTopic.DepthImage", "BD_Roamer/depth_camera/image");
    declare_parameter("OctoMap.Resolution", 0.05);
    declare_parameter("OctoMap.FrameId", "octomap");
    std::string rgb_img_topic, depth_img_topic, octomap_frame_id;
    double octomap_resolution;
    get_parameter("InputTopic.RgbImage", rgb_img_topic);
    get_parameter("InputTopic.DepthImage", depth_img_topic);
    get_parameter("OctoMap.Resolution", octomap_resolution);
    get_parameter("OctoMap.FrameId", octomap_frame_id);

    std::string this_package_path = ament_index_cpp::get_package_share_directory("orb_slam3");
    std::string setting_file = this_package_path + "/config/settings/SlamConfig.yaml";

    mSlamer = std::make_unique<ORB_SLAM3::System>(this_package_path + "/config/settings/ORBvoc.txt",
                                                  this_package_path + "/config/settings/SlamConfig.yaml",
                                                  ORB_SLAM3::System::RGBD,
                                                  this_package_path + "/model/semantic_segmentation.trt");

    Eigen::Transform<float, 3, Eigen::Affine> tmp_tran = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
    tmp_tran.rotate(Eigen::AngleAxis<float>(M_PI / 2, Eigen::Vector3f::UnitZ()));
    tmp_tran.rotate(Eigen::AngleAxis<float>(-M_PI / 2, Eigen::Vector3f::UnitY()));
    mTrviz = tmp_tran.matrix().inverse();
    tmp_tran = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
    mTodom = tmp_tran.matrix();
    mTcr = tmp_tran.inverse();

    /* Initiate subscriptions, publishers, timers and tf */
    mRgbSub = new message_filters::Subscriber<sensor_msgs::msg::Image>(this, rgb_img_topic);
    mDepthSub = new message_filters::Subscriber<sensor_msgs::msg::Image>(this, depth_img_topic);
    mSynchronizer = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *mRgbSub, *mDepthSub);

    mSynchronizer->registerCallback(
        std::bind(&SlamNode::rgbDepthCallback, this, std::placeholders::_1, std::placeholders::_2));

    // mMutiThreadCallback = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    // mSingleThreadCallback = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    mImuSub =
        create_subscription<sensor_msgs::msg::Imu>("BD_Roamer/imu",
                                                   rclcpp::SensorDataQoS().best_effort(),
                                                   std::bind(&SlamNode::imuCallback, this, std::placeholders::_1));

    mCloudMapPub =
        create_publisher<sensor_msgs::msg::PointCloud2>("orb_slam3/cloud_map", rclcpp::SensorDataQoS().reliable());
    mOdomPub = create_publisher<nav_msgs::msg::Odometry>("orb_slam3/odom", rclcpp::ServicesQoS().reliable());
    mPathPub = create_publisher<nav_msgs::msg::Path>("orb_slam3/odom_path", rclcpp::SensorDataQoS().reliable());

    mTfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // mTimer[0] = create_wall_timer(
    //     std::chrono::milliseconds(200), std::bind(&SlamNode::publishCloudMap, this), mMutiThreadCallback);
    // mTimer[1] = create_wall_timer(
    //     std::chrono::milliseconds(100), std::bind(&SlamNode::publishNavMsgs, this), mMutiThreadCallback);
    mTimer[0] = create_wall_timer(std::chrono::milliseconds(200), std::bind(&SlamNode::publishCloudMap, this));
    mTimer[1] = create_wall_timer(std::chrono::milliseconds(180), std::bind(&SlamNode::publishNavMsgs, this));
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
    catch (const cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        // cv_ptrD = cv_bridge::toCvCopy(depth_msg, "16UC1");32FC1
        cv_ptrD = cv_bridge::toCvShare(depth_msg);
    }
    catch (const cv_bridge::Exception& e)
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
        mTcw = mSlamer->TrackRGBD(rgb_img, depth_img, getSec(cv_ptrRGB->header.stamp), mImuMeas);
        // mTcw = mSlamer->TrackRGBD(rgb_img, depth_img, getSec(cv_ptrRGB->header.stamp));
        mImuMeas.clear();
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "The size of rgb image and depth image are not equal.");
        std::exit(EXIT_FAILURE);
    }
}

void SlamNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg)
{
    ORB_SLAM3::IMU::Point point(imu_msg->linear_acceleration.x,
                                imu_msg->linear_acceleration.y,
                                imu_msg->linear_acceleration.z,
                                imu_msg->angular_velocity.x,
                                imu_msg->angular_velocity.y,
                                imu_msg->angular_velocity.z,
                                getSec(imu_msg->header.stamp));
    mImuMeas.emplace_back(point);
}

void SlamNode::publishCloudMap()
{
    auto temp_cloud_map = mSlamer->getPointCloudMap();
    if (temp_cloud_map->empty()) return;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*temp_cloud_map, *cloud_map, mTrviz);

    sensor_msgs::msg::PointCloud2 cloud_map_msg;
    pcl::toROSMsg(*cloud_map, cloud_map_msg);
    cloud_map_msg.header.stamp = this->now();
    cloud_map_msg.header.frame_id = "cloud_map";
    mCloudMapPub->publish(cloud_map_msg);
}

void SlamNode::publishNavMsgs()
{
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

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = "cloud_map";
    odom_msg.child_frame_id = "odom";
    odom_msg.header.stamp = this->now();
    odom_msg.pose.pose.position.x = matrix_cw(0, 3);
    odom_msg.pose.pose.position.y = matrix_cw(1, 3);
    odom_msg.pose.pose.position.z = matrix_cw(2, 3);
    odom_msg.pose.pose.orientation.x = quaternion_wc.x();
    odom_msg.pose.pose.orientation.y = quaternion_wc.y();
    odom_msg.pose.pose.orientation.z = quaternion_wc.z();
    odom_msg.pose.pose.orientation.w = quaternion_wc.w();
    mOdomPub->publish(odom_msg);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = odom_msg.header;
    pose_msg.pose = odom_msg.pose.pose;
    mPathMsg.header.stamp = odom_msg.header.stamp;
    mPathMsg.header.frame_id = "cloud_map";
    mPathMsg.poses.push_back(pose_msg);
    mPathPub->publish(mPathMsg);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id = "cloud_map";
    tf_msg.child_frame_id = "odom";
    tf_msg.header.stamp = odom_msg.header.stamp;
    tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
    tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
    tf_msg.transform.translation.z = odom_msg.pose.pose.position.z;
    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
    mTfBroadcaster->sendTransform(tf_msg);
}
