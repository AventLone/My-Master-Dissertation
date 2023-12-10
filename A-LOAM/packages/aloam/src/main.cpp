#include "SystemNode.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto system_node = std::make_shared<SystemNode>("aloam_system");
    rclcpp::spin(system_node);
    rclcpp::shutdown();
    return 0;
}