#include "Controller.h"
#include "LocalPlanner.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto controller_node = std::make_shared<Controller>("controller");
    auto local_planner_node = std::make_shared<LocalPlanner>("local_planner");

    rclcpp::ExecutorOptions options;
    rclcpp::executors::MultiThreadedExecutor executor(options, 2);   // Assign 2 threads to the thread pool.
    executor.add_node(controller_node);
    executor.add_node(local_planner_node);

    executor.spin();
    rclcpp::shutdown();

    return 0;
}