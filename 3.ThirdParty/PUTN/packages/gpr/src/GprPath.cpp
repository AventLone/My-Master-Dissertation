#include "GprPath.h"

GprPath::GprPath(const std::string& name) : Node(name)
{
    declare_parameter("HyperParameters.l", 1.1284);
    declare_parameter("HyperParameters.f", 0.4048);
    declare_parameter("HyperParameters.n", 0.1033);
    get_parameter("HyperParameters.l", mLengthScale);
    get_parameter("HyperParameters.f", mSigma_f);
    get_parameter("HyperParameters.n", mSigma_n);


    mTreeSub = create_subscription<MultiArray>("putn/global_planning/tree_tra",
                                               rclcpp::ParametersQoS().reliable(),
                                               std::bind(&GprPath::treeCallBack, this, std::placeholders::_1));
    mPathSub = create_subscription<MultiArray>("putn/global_planning/global_path",
                                               rclcpp::ParametersQoS().reliable(),
                                               std::bind(&GprPath::pathCallBack, this, std::placeholders::_1));

    mSurfacePredictPub = create_publisher<MultiArray>("putn/gpr/surface_prediction", rclcpp::ServicesQoS().reliable());
}

void GprPath::treeCallBack(const MultiArray::ConstSharedPtr& msg)
{
    if (msg->data.empty()) return;

    int num = static_cast<int>(msg->data.size() / 4);
    for (int i = 0; i < num; i++)
    {
        Eigen::Vector3f tmp_in;
        tmp_in << msg->data[4 * i], msg->data[4 * i + 1], msg->data[4 * i + 2];
        mTrainInputs.push_back(tmp_in);
        Eigen::Matrix<float, 1, 1> tmp_out;
        tmp_out << msg->data[4 * i + 3];
        mTrainOutputs.push_back(tmp_out);
    }

    GaussianProcessRegression<float> myGPR(mInputDim, mOutputDim);
    myGPR.setHyperParams(mLengthScale, mSigma_f, mSigma_n);

    for (size_t k = 0; k < mTrainInputs.size(); ++k)
    {
        myGPR.addTrainingData(mTrainInputs[k], mTrainOutputs[k]);
    }

    double threshold = 0.1;

    if (mTestInputs.empty())
    {
        mTrainInputs.clear();
        mTrainOutputs.clear();
        mTestInputs.clear();
        mTestOutputs.clear();
    }

    MultiArray surface_prediction_msg;
    // MultiArray out_ys;

    for (size_t k = 0; k < mTestInputs.size(); ++k)
    {
        float output_covariance;   // covariance
        auto outp = myGPR.doRegression(mTestInputs[k], output_covariance);
        Eigen::Matrix<float, 1, 1> tmp_out;
        tmp_out << outp;
        mTestOutputs.push_back(tmp_out);


        // auto output_covariance = myGPR.doRegressioncov(mTestInputs[k]);
        surface_prediction_msg.data.push_back(mTestInputs[k](0));
        surface_prediction_msg.data.push_back(mTestInputs[k](1));
        surface_prediction_msg.data.push_back(mTestInputs[k](2));
        surface_prediction_msg.data.push_back(tmp_out(0, 0));
        surface_prediction_msg.data.push_back(output_covariance);
    }
    mSurfacePredictPub->publish(surface_prediction_msg);

    mTrainInputs.clear();
    mTrainOutputs.clear();
    mTestInputs.clear();
    mTestOutputs.clear();
}

void GprPath::pathCallBack(const MultiArray::ConstSharedPtr& msg)
{
    if (msg->data.empty()) return;

    int num = static_cast<int>(msg->data.size() / 3);
    for (int i = 0; i < num; ++i)
    {
        Eigen::Vector3f tmp_in;
        tmp_in << msg->data[3 * i], msg->data[3 * i + 1], msg->data[3 * i + 2];
        mTestInputs.push_back(tmp_in);
    }
}
