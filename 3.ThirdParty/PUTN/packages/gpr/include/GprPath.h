#pragma once
#include "GaussianProcessRegression.hpp"
#include <fstream>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>

class GprPath : public rclcpp::Node
{
    using MultiArray = std_msgs::msg::Float32MultiArray;

public:
    explicit GprPath(const std::string& name);

    ~GprPath() = default;

private:
    double mLengthScale, mSigma_f, mSigma_n;   // Hyper parameters

    const size_t mInputDim{2}, mOutputDim{1};
    
    std::vector<Eigen::Vector3f> mTrainInputs, mTestInputs;
    std::vector<Eigen::Matrix<float, 1, 1>> mTrainOutputs, mTestOutputs;
    rclcpp::Subscription<MultiArray>::SharedPtr mTreeSub, mPathSub;
    rclcpp::Publisher<MultiArray>::SharedPtr mSurfacePredictPub;

private:
    void treeCallBack(const MultiArray::ConstSharedPtr& msg);
    void pathCallBack(const MultiArray::ConstSharedPtr& msg);
};