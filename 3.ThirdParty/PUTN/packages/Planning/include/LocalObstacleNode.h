#include <tf2_ros/transform_listener.h>
#include "backward.hpp"
#include "putnDataType.h"
#include <pcl/filters/passthrough.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace backward
{
backward::SignalHandling sh;
}

class LocalObstacleNode : rclcpp::Node
{
public:
    explicit LocalObstacleNode(const std::string& name);

private:
    double resolution, local_x_l, local_x_u, local_y_l, local_y_u, local_z_l, local_z_u;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mMapSub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mObstacleVisPub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mObstaclePub;

    tf2::BufferCore mTfBuffer;
    tf2_ros::TransformListener mTfListener{mTfBuffer};

    void mapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
};