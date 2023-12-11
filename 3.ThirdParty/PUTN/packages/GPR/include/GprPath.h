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
    double length_scale;
    double sigma_f;
    double sigma_n;
    const size_t input_dim{2}, output_dim{1};
    GaussianProcessRegression<float> mGPR{1, 1};
    std::vector<Eigen::Vector3f> mTrainInputs, mTrainOutputs, mTestInputs, mTestOutputs;

    rclcpp::Publisher<MultiArray>::SharedPtr mSurfacePredictPub;
    rclcpp::Subscription<MultiArray>::SharedPtr mTreeSub, mPathSub;

private:
    void treeCallBack(const MultiArray::ConstPtr& msg);
    void pathCallBack(const MultiArray::ConstPtr& msg);
};


// string filepath;

// template<typename input_type, typename output_type>
// void load_data(const char* fname, std::vector<input_type>& inputs, std::vector<output_type>& outputs, int input_dim,
//                int output_dim)
// {
//     std::cout << "entry this branch........" << std::endl;
//     input_type inp, tinp;
//     output_type outp, toutp;
//     std::ifstream myfile(fname);
//     if (!myfile.is_open())
//     {
//         std::cout << "Fail to open the file" << std::endl;
//         return;
//     }
//     std::string line;
//     while (getline(myfile, line))
//     {
//         std::cout << line << " ";
//         std::istringstream line_stream(line);
//         for (size_t k = 0; k < input_dim; k++) line_stream >> inp(k);
//         for (size_t k = 0; k < output_dim; k++) line_stream >> outp(k);
//         inputs.push_back(inp);
//         outputs.push_back(outp);
//     }
//     std::cout << "finish loading..." << std::endl;
// }


// void clearVectorIn(std::vector<Eigen::Vector3f>& vt)
// {
//     std::vector<Eigen::Vector3f> veTemp;
//     veTemp.swap(vt);
// }

// void clearVectorOut(std::vector<Eigen::Vector3f>& vt)
// {
//     std::vector<Eigen::Vector3f> veTemp;
//     veTemp.swap(vt);
// }

// void treeCallBack(const std_msgs::msg::Float32MultiArray::ConstPtr& msg)
// {
//     double dur;
//     clock_t start, end;
//     start = std::clock();
//     ROS_INFO("[node] receive the tree");

//     if (msg->data.size() == 0) return;

//     int num = (int)(msg->data.size() / 4);
//     for (int i = 0; i < num; i++)
//     {
//         input_type tmp_in;
//         tmp_in << msg->data[4 * i], msg->data[4 * i + 1], msg->data[4 * i + 2];
//         train_inputs.push_back(tmp_in);
//         output_type tmp_out;
//         tmp_out << msg->data[4 * i + 3];
//         train_outputs.push_back(tmp_out);
//     }

//     // GPr
//     GaussianProcessRegression<float> myGPR(input_dim, output_dim);
//     setHyperParameters(filepath.c_str(), myGPR);

//     for (size_t k = 0; k < train_inputs.size(); k++)
//     {
//         myGPR.addTrainingData(train_inputs[k], train_outputs[k]);
//     }

//     double threshold = 0.1;


//     if (test_inputs.size() == 0)
//     {
//         clearVectorIn(train_inputs);
//         clearVectorIn(test_inputs);
//         clearVectorOut(train_outputs);
//         clearVectorOut(test_outputs);
//     }


//     std_msgs::Float32MultiArray out_ym;
//     std_msgs::Float32MultiArray out_ys;

//     for (size_t k = 0; k < test_inputs.size(); k++)
//     {
//         auto outp = myGPR.doRegression(test_inputs[k]);
//         output_type tmp_out;
//         tmp_out << outp;
//         test_outputs.push_back(tmp_out);
//         // covariance
//         auto outp_cov = myGPR.doRegressioncov(test_inputs[k]);
//         out_ym.data.push_back(test_inputs[k](0, 0));
//         out_ym.data.push_back(test_inputs[k](1, 0));
//         out_ym.data.push_back(test_inputs[k](2, 0));
//         out_ym.data.push_back(tmp_out(0, 0));
//         out_ym.data.push_back(outp_cov(0, 0));
//     }
//     _surf_predict_pub.publish(out_ym);
//     std_msgs::Float32MultiArray tmp_out;
//     clearVectorIn(train_inputs);
//     clearVectorIn(test_inputs);
//     clearVectorOut(train_outputs);
//     clearVectorOut(test_outputs);
//     end = clock();
//     dur = (double)(end - start);
//     cout << "Time consume : " << dur / 1000 << " ms" << endl;
// }

// void pathCallBack(const std_msgs::Float32MultiArray::ConstPtr& msg)
// {
//     ROS_INFO("[node] receive the path");
//     if (msg->data.size() == 0) return;

//     int num = (int)(msg->data.size() / 3);
//     for (int i = 0; i < num; i++)
//     {
//         input_type tmp_in;
//         tmp_in << msg->data[3 * i], msg->data[3 * i + 1], msg->data[3 * i + 2];
//         test_inputs.push_back(tmp_in);
//     }
// }
