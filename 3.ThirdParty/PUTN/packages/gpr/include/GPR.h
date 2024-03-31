// #pragma once
// #include <Eigen/Dense>
// #include <iostream>
// #include <vector>

// class GPR
// {
// public:
//     explicit GPR(int inputDim, int outputDim)
//     {
//         mInputData.resize(inputDim, 0);
//         mOutputData.resize(outputDim, 0);
//         n_data_ = 0;
//     }

//     void setHyperParams(double l, double f, double n)
//     {
//         mLengthScale = l;
//         mSigma_f = f;
//         mSigma_n = n;
//     }
//     void getHyperParams(double& l, double& f, double& n)
//     {
//         l = mLengthScale;
//         f = mSigma_f;
//         n = mSigma_n;
//         std::cout << l << " " << f << " " << n;
//     }

//     /* add data one by one */
//     void addTrainingData(const Eigen::Vector3f& newInput, float newOutput);

//     // batch add data
//     void addTrainingDataBatch(const Eigen::MatrixXf& newInput, const Eigen::MatrixXf& newOutput);

//     float SQEcovFuncD(Eigen::VectorXf x1, Eigen::VectorXf x2);

//     void debug();

//     Eigen::VectorXf SQEcovFunc(Eigen::MatrixXf x1);
//     Eigen::VectorXf SQEcovFunc(Eigen::MatrixXf x1, Eigen::VectorXf x2);
//     // these are fast methods
//     void prepareRegression(bool force_prepare = false);

//     Eigen::VectorXf doRegression(const Eigen::VectorXf& inp, bool prepare = false);

//     Eigen::VectorXf doRegressioncov(const Eigen::VectorXf& inp, bool prepare = false);

//     // these are the old implementations that are slow, inaccurate and easy to understand
//     void prepareRegressionOld(bool force_prepare = false);

//     Eigen::VectorXf doRegressionOld(const Eigen::VectorXf& inp, bool prepare = false);

//     int get_n_data()
//     {
//         return n_data_;
//     }
//     const Eigen::MatrixXf& getInputData()
//     {
//         return mInputData;
//     }
//     const Eigen::MatrixXf& getOutputData()
//     {
//         return mOutputData;
//     }

//     void clearTrainingData();

// private:
//     Eigen::MatrixXf mInputData;
//     Eigen::MatrixXf mOutputData;
//     Eigen::MatrixXf KXX;   // covariance between the training data
//     Eigen::MatrixXf KXX_;
//     Eigen::VectorXf KXx;   // covariance between the training data and test data

//     int n_data_;
//     bool b_need_prepare_;

//     double mLengthScale;
//     double mSigma_f;
//     double mSigma_n;

//     Eigen::VectorXf mDist, mRegressors;
//     Eigen::MatrixXf mAlpha;
// };
