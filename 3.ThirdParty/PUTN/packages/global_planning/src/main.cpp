#include "GlobalPlanningNode.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto global_planning_node = std::make_shared<GlobalPlanningNode>("global_planner");
    while (rclcpp::ok())
    {
        geometry_msgs::msg::TransformStamped transform;
        while (rclcpp::ok())
        {
            try
            {
                transform = global_planning_node->mTfBuffer->lookupTransform("world", "base_link", tf2::TimePointZero);
                break;
            }
            catch (const tf2::TransformException& e)
            {
                continue;
            }
        }
        global_planning_node->setStartPoint(transform);
        rclcpp::spin_some(global_planning_node);
        global_planning_node->callPlanner();
    }
    // rclcpp::ExecutorOptions options;
    // rclcpp::executors::MultiThreadedExecutor executor(options, 2);   // Assign 2 threads to the thread pool.
    rclcpp::shutdown();
    return 0;
}