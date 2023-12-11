#include "GlobalPlanningNode.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto global_planning_node = std::make_shared<GlobalPlanningNode>("global_planning");
    rclcpp::spin(global_planning_node);
    rclcpp::shutdown();
    return 0;
}