#include "WayPointGenerator.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto waypoint_generator_node = std::make_shared<WayPointGenerator>("waypoint_generator");
    rclcpp::spin(waypoint_generator_node);
    rclcpp::shutdown();
    return 0;
}