#include "SlamNode.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto slam_node = std::make_shared<SlamNode>("orb_slam3_rgbd");
    // auto occupancy_mapping_node = std::make_shared<OccupancyMapping>("occupancy_mapping");
    // slam_node->setOccupancyMapper(occupancy_mapping_node);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(slam_node);
    // executor.add_node(map_node);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}