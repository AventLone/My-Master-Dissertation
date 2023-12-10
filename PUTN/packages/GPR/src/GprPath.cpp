#include "GprPath.h"

GprPath::GprPath(const std::string& name) : Node(name)
{
    mSurfacePredictPub = create_publisher<MultiArray>("putn/surface_prediction", rclcpp::ParametersQoS().reliable());
    mTreeSub = create_subscription<MultiArray>("putn/global_planning/tree_tra",
                                               rclcpp::ParametersQoS().reliable(),
                                               std::bind(&GprPath::treeCallBack, this, std::placeholders::_1));
    mPathSub = create_subscription<MultiArray>("putn/global_planning/tree_tra",
                                               rclcpp::ParametersQoS().reliable(),
                                               std::bind(&GprPath::pathCallBack, this, std::placeholders::_1));
}

void GprPath::treeCallBack(const MultiArray::ConstPtr& msg)
{
    double duration;
    clock_t start, end;
    start = std::clock();
    RCLCPP_INFO(get_logger(), "Receive the tree.");

    if (msg->data.size() == 0) return;

    int num = static_cast<int>(msg->data.size() / 4);
    for (int i = 0; i < num; i++)
    {
        Eigen::Vector3f tmp_in;
        tmp_in << msg->data[4 * i], msg->data[4 * i + 1], msg->data[4 * i + 2];
        mTrainInputs.push_back(tmp_in);
        Eigen::Vector3f tmp_out;
        tmp_out << msg->data[4 * i + 3];
        mTrainOutputs.push_back(tmp_out);
    }

    GaussianProcessRegression<float> myGPR(input_dim, output_dim);
    setHyperParameters(filepath.c_str(), myGPR);

    for (size_t k = 0; k < mTrainInputs.size(); ++k)
    {
        myGPR.addTrainingData(mTrainInputs[k], mTrainOutputs[k]);
    }

    double threshold = 0.1;


    if (mTestInputs.size() == 0)
    {
        mTrainInputs.clear();
        mTrainOutputs.clear();
        mTestInputs.clear();
        mTestOutputs.clear();
    }


    MultiArray out_ym;
    MultiArray out_ys;

    for (size_t k = 0; k < mTestInputs.size(); ++k)
    {
        auto outp = myGPR.doRegression(mTestInputs[k]);
        Eigen::Vector3f tmp_out;
        tmp_out << outp;
        mTestOutputs.push_back(tmp_out);
        // covariance
        auto outp_cov = myGPR.doRegressioncov(mTestInputs[k]);
        out_ym.data.push_back(mTestInputs[k](0, 0));
        out_ym.data.push_back(mTestInputs[k](1, 0));
        out_ym.data.push_back(mTestInputs[k](2, 0));
        out_ym.data.push_back(tmp_out(0, 0));
        out_ym.data.push_back(outp_cov(0, 0));
    }
    mSurfacePredictPub->publish(out_ym);
    MultiArray tmp_out;

    mTrainInputs.clear();
    mTrainOutputs.clear();
    mTestInputs.clear();
    mTestOutputs.clear();

    end = std::clock();
    duration = static_cast<double>(end - start);

    RCLCPP_DEBUG(get_logger(), "Time consume : %f ms", duration / 1000.0);
}

void GprPath::pathCallBack(const MultiArray::ConstPtr& msg)
{
    RCLCPP_INFO(get_logger(), "[node] receive the path");
    if (msg->data.size() == 0) return;

    int num = static_cast<int>(msg->data.size() / 3);
    for (int i = 0; i < num; ++i)
    {
        Eigen::Vector3f tmp_in;
        tmp_in << msg->data[3 * i], msg->data[3 * i + 1], msg->data[3 * i + 2];
        mTestInputs.push_back(tmp_in);
    }
}
