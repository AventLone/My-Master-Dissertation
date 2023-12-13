#include "Mapping.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto mapping_node = std::make_shared<faster_lio::Mapping>("faster_lio_mapping");

    rclcpp::spin(mapping_node);
    rclcpp::shutdown();

    return 0;
}