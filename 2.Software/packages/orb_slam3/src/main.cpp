#include "SlamNode.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto slam_node = std::make_shared<SlamNode>("orb_slam3");
    rclcpp::ExecutorOptions options;
    rclcpp::executors::MultiThreadedExecutor executor(options, 2);
    executor.add_node(slam_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}