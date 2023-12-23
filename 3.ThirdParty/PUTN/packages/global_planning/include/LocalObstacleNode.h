#include "api/DataTypes.h"
#include <tf2_ros/transform_listener.h>
#include <pcl/filters/passthrough.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

/**
 * @brief Detect local obstacles.
 */
class LocalObstacleNode : public rclcpp::Node
{
public:
    explicit LocalObstacleNode(const std::string& name);

private:
    double mResolution, mLocal_x_l, mLocal_x_u, mLocal_y_l, mLocal_y_u, mLocal_z_l, mLocal_z_u;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mScanSub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mObstacleVisPub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mObstaclePub;

    std::unique_ptr<tf2_ros::Buffer> mTfBuffer{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> mTfListener{nullptr};

    void scanCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
};