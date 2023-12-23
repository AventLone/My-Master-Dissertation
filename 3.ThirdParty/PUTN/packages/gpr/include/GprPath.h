#pragma once
#include "GaussianProcessRegression.hpp"
#include <fstream>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>

template<typename R>
static void setHyperParameters(const char* fname, GaussianProcessRegression<R>& gpr)
{
    std::ifstream myfile;
    myfile.open(fname);
    if (!myfile.is_open())
    {
        std::cout << "Fail to open the file" << std::endl;
        return;
    }
    R l, f, n;
    myfile >> l >> f >> n;
    myfile.close();
    gpr.setHyperParams(l, f, n);
}

class GprPath : public rclcpp::Node
{
    using MultiArray = std_msgs::msg::Float32MultiArray;

public:
    explicit GprPath(const std::string& name);

    ~GprPath() = default;

private:
    std::string mFilePath;

    double mLengthScale;
    double sigma_f;
    double sigma_n;
    const size_t mInputDim{2}, mOutputDim{1};
    GaussianProcessRegression<float> mGPR{1, 1};
    std::vector<Eigen::Vector3f> mTrainInputs, mTrainOutputs, mTestInputs, mTestOutputs;

    rclcpp::Subscription<MultiArray>::SharedPtr mTreeSub, mPathSub;
    rclcpp::Publisher<MultiArray>::SharedPtr mSurfacePredictPub;

private:
    void treeCallBack(const MultiArray::ConstSharedPtr& msg);
    void pathCallBack(const MultiArray::ConstSharedPtr& msg);
};