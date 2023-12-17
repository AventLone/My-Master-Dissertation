#include "GlobalPlanningNode.h"


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto global_planning_node = std::make_shared<GlobalPlanningNode>("global_planner");

    auto task_globalPlanning = [&]() -> void
    {
        while (rclcpp::ok())
        {
            while (rclcpp::ok())
            {
                try
                {
                    global_planning_node;
                    break;
                }
                catch (const std::exception& e)
                {
                    std::cerr << e.what() << '\n';
                    continue;
                }
            }
            rclcpp::spin_some(global_planning_node);
            global_planning_node->callPlanner();
        }
    };

    // std::thread my_thread(task_globalPlanning);

    // rclcpp::ExecutorOptions options;
    // rclcpp::executors::MultiThreadedExecutor executor(options, 2);   // Assign 2 threads to the thread pool.

    // executor.add_node(global_planning_node);

    // executor.spin();

    rclcpp::shutdown();

    // my_thread.join();

    return 0;
}