#include "GlobalPlanningNode.h"
#include "LocalObstacleNode.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto global_planning_node = std::make_shared<GlobalPlanningNode>("global_planner");
    auto local_obstacle_node = std::make_shared<LocalObstacleNode>("local_obstacle");

    auto run = [&]() -> void
    {
        while (rclcpp::ok())
        {
            geometry_msgs::msg::TransformStamped transform;
            while (rclcpp::ok())
            {
                try
                {
                    transform =
                        global_planning_node->mTfBuffer->lookupTransform("world", "base_link", tf2::TimePointZero);
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
    };

    std::thread thread_1(run);
    rclcpp::spin(local_obstacle_node);

    // rclcpp::ExecutorOptions options;
    // rclcpp::executors::MultiThreadedExecutor executor(options, 2);   // Assign 2 threads to the thread pool.
    rclcpp::shutdown();
    if (thread_1.joinable())
    {
        thread_1.join();
    }
    return 0;
}