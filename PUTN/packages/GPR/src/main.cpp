#include "GaussianProcessRegression.hpp"
#include "GprPath.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<GprPath> gpr_path = std::make_shared<GprPath>("gpr_path");
    rclcpp::spin(gpr_path);
    rclcpp::shutdown();
    return 0;
}