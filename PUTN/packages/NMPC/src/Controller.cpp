#include "Controller.h"

Controller::Controller(const std::string& name) : rclcpp::Node(name)
{
    mLocalPlan = Eigen::MatrixXd::Zero(N, 2);
}

void Controller::localPlanCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg)
{
    for (int i = 0; i < N; ++i)
    {
        mLocalPlan(i, 0) = msg->data[2 * i];
        mLocalPlan(i, 1) = msg->data[2 * i + 1];
    }
}