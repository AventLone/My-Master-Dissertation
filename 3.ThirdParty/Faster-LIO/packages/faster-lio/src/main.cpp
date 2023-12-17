#include "Mapping.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto mapping_node = std::make_shared<faster_lio::Mapping>("faster_lio");

    while (rclcpp::ok())
    {
        rclcpp::spin_some(mapping_node);
        mapping_node->run();
        rclcpp::sleep_for(std::chrono::nanoseconds(100));
    }

    // rclcpp::spin(mapping_node);
    rclcpp::shutdown();

    return 0;
}