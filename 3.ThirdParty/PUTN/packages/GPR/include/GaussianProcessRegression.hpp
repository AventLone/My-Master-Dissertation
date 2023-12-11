#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include "macro_print.h"

template<typename RealType>
class GaussianProcessRegression
{
    using MatrixXr = Eigen::Matrix<RealType, Eigen::Dynamic, Eigen::Dynamic>;
    using VectorXr = Eigen::Matrix<RealType, Eigen::Dynamic, 1>;

public:
    explicit GaussianProcessRegression(int inputDim, int outputDim);

    void setHyperParams(double l, double f, double n)
    {
        mLengthScale = l;
        mSigma_f = f;
        mSigma_n = n;
    }
    void getHyperParams(double& l, double& f, double& n)
    {
        l = mLengthScale;
        f = mSigma_f;
        n = mSigma_n;
        std::cout << l << " " << f << " " << n;
    }

    // add data one by one
    void addTrainingData(const VectorXr& newInput, const VectorXr& newOutput);

    // batch add data
    void addTrainingDataBatch(const MatrixXr& newInput, const MatrixXr& newOutput);

    RealType SQEcovFuncD(VectorXr x1, VectorXr x2);

    void debug();

    MatrixXr SQEcovFunc(MatrixXr x1);
    VectorXr SQEcovFunc(MatrixXr x1, VectorXr x2);
    // these are fast methods
    void prepareRegression(bool force_prepare = false);

    VectorXr doRegression(const VectorXr& inp, bool prepare = false);

    VectorXr doRegressioncov(const VectorXr& inp, bool prepare = false);

    // these are the old implementations that are slow, inaccurate and easy to understand
    void prepareRegressionOld(bool force_prepare = false);

    VectorXr doRegressionOld(const VectorXr& inp, bool prepare = false);

    int get_n_data()
    {
        return n_data_;
    }
    const MatrixXr& getInputData()
    {
        return mInputData;
    }
    const MatrixXr& getOutputData()
    {
        return mOutputData;
    }

    void clearTrainingData();

private:
    MatrixXr mInputData;
    MatrixXr mOutputData;
    MatrixXr KXX;   // covariance between the training data
    MatrixXr KXX_;
    VectorXr KXx;   // covariance between the training data and test data

    int n_data_;
    bool b_need_prepare_;

    double mLengthScale;
    double mSigma_f;
    double mSigma_n;

    VectorXr mDist, mRegressors;
    MatrixXr mAlpha;
};

template<typename R>
GaussianProcessRegression<R>::GaussianProcessRegression(int inputDim, int outputDim)
{
    mInputData.resize(inputDim, 0);
    mOutputData.resize(outputDim, 0);
    n_data_ = 0;
}

template<typename R>
void GaussianProcessRegression<R>::addTrainingData(const VectorXr& newInput, const VectorXr& newOutput)
{
    n_data_++;
    if (n_data_ >= mInputData.cols())
    {
        mInputData.conservativeResize(mInputData.rows(), n_data_);
        mOutputData.conservativeResize(mOutputData.rows(), n_data_);
    }
    mInputData.col(n_data_ - 1) = newInput;
    mOutputData.col(n_data_ - 1) = newOutput;
    b_need_prepare_ = true;
}

template<typename R>
void GaussianProcessRegression<R>::addTrainingDataBatch(const MatrixXr& newInput, const MatrixXr& newOutput)
{
    // sanity check of provided data
    assert(newInput.cols() == newOutput.cols());
    // if this is the first data, just add it..
    if (n_data_ == 0)
    {
        mInputData = newInput;
        mOutputData = newOutput;
        n_data_ = mInputData.cols();
    }
    // if we already have data, first check dimensionaly match
    else
    {
        assert(mInputData.rows() == newInput.rows());
        assert(mOutputData.rows() == newOutput.rows());
        size_t n_data_old = n_data_;
        n_data_ += newInput.cols();
        // resize the matrices
        if (n_data_ > mInputData.cols())
        {
            mInputData.conservativeResize(mInputData.rows(), n_data_);
            mOutputData.conservativeResize(mOutputData.rows(), n_data_);
        }
        // insert the new data using block operations
        mInputData.block(0, n_data_old, newInput.rows(), newInput.cols()) = newInput;
        mOutputData.block(0, n_data_old, newOutput.rows(), newOutput.cols()) = newOutput;
    }
    // in any case after adding a batch of data we need to recompute decomposition (in lieu of matrix inversion)
    b_need_prepare_ = true;
}

// gaussian kernel fuction for compute covariance
template<typename R>
R GaussianProcessRegression<R>::SQEcovFuncD(VectorXr x1, VectorXr x2)
{
    mDist = x1 - x2;
    double d = mDist.dot(mDist);
    d = mSigma_f * mSigma_f * exp(-1 / mLengthScale / mLengthScale / 2 * d);
    return d;
}

template<typename R>
typename GaussianProcessRegression<R>::VectorXr GaussianProcessRegression<R>::SQEcovFunc(MatrixXr x1, VectorXr x2)
{
    int nCol = x1.cols();
    VectorXr KXx(nCol);
    for (int i = 0; i < nCol; i++)
    {
        KXx(i) = SQEcovFuncD(x1.col(i), x2);
    }
    return KXx;
}


template<typename R>
void GaussianProcessRegression<R>::prepareRegression(bool force_prepare)
{
    if (!b_need_prepare_ & !force_prepare) return;

    KXX = SQEcovFunc(mInputData);
    KXX_ = KXX;
    // add measurement noise
    // we set noise to zero
    for (int i = 0; i < KXX.cols(); i++) KXX_(i, i) += mSigma_n * mSigma_n;
    mAlpha.resize(mOutputData.rows(), mOutputData.cols());
    // pretty slow decomposition to compute
    // Eigen::FullPivLU<MatrixXr> decomposition(KXX_);
    // this is much much faster:
    Eigen::LDLT<MatrixXr> decomposition(KXX_);
    for (size_t i = 0; i < mOutputData.rows(); ++i)
    {
        mAlpha.row(i) = (decomposition.solve(mOutputData.row(i).transpose())).transpose();
    }
    b_need_prepare_ = false;
}

// This is a slow and and deprecated version that is easier to understand.
template<typename R>
void GaussianProcessRegression<R>::prepareRegressionOld(bool force_prepare)
{
    if (!b_need_prepare_ & !force_prepare) return;

    KXX = SQEcovFunc(mInputData);
    KXX_ = KXX;

    // add measurement noise
    for (int i = 0; i < KXX.cols(); i++) KXX_(i, i) += mSigma_n * mSigma_n;

    // this is a time theif:
    KXX_ = KXX_.inverse();
    b_need_prepare_ = false;
}

// This is the right way to do it but this code should be refactored and tweaked so that the decompositon is not
// recomputed unless new training data has arrived.
template<typename R>
typename GaussianProcessRegression<R>::VectorXr GaussianProcessRegression<R>::doRegression(const VectorXr& inp,
                                                                                           bool prepare)
{
    // if(prepare || b_need_prepare_){
    //   prepareRegression();
    // }
    // can return immediately if no training data has been added..
    VectorXr outp(mOutputData.rows());
    outp.setZero();
    if (n_data_ == 0) return outp;

    prepareRegression(prepare);
    // ok

    outp.setZero();
    KXx = SQEcovFunc(mInputData, inp);
    // std::cout<<"KXx  "<<KXx<<std::endl;
    for (size_t i = 0; i < mOutputData.rows(); ++i) outp(i) = KXx.dot(mAlpha.row(i));

    MatrixXr outp1 = SQEcovFunc(inp, inp) - KXx * KXX * (KXx.transpose());
    // R output2  =  SQEcovFunc(inp,inp) -  KXx*KXX* KXx.transpose();

    return outp;
}

template<typename R>
typename GaussianProcessRegression<R>::VectorXr GaussianProcessRegression<R>::doRegressioncov(const VectorXr& inp,
                                                                                              bool prepare)
{
    // if(prepare || b_need_prepare_){
    //   prepareRegression();
    // }
    // can return immediately if no training data has been added..
    VectorXr outp(mOutputData.rows());
    outp.setZero();
    if (n_data_ == 0) return outp;

    prepareRegression(prepare);
    // ok

    outp.setZero();
    KXx = SQEcovFunc(mInputData, inp);
    // std::cout<<"KXx  "<<KXx<<std::endl;
    for (size_t i = 0; i < mOutputData.rows(); ++i) outp(i) = KXx.dot(mAlpha.row(i));

    MatrixXr outp1 = SQEcovFunc(inp, inp) - KXx * KXX * (KXx.transpose());
    // R output2  =  SQEcovFunc(inp,inp) -  KXx*KXX* KXx.transpose();

    return outp1;
}

template<typename R>
typename GaussianProcessRegression<R>::VectorXr GaussianProcessRegression<R>::doRegressionOld(const VectorXr& inp,
                                                                                              bool prepare)
{
    if (prepare || b_need_prepare_)
    {
        prepareRegressionOld();
    }
    VectorXr outp(mOutputData.rows());
    outp.setZero();
    KXx = SQEcovFunc(mInputData, inp);
    VectorXr tmp(mInputData.cols());

    // this line is the slow one, hard to speed up further?
    tmp = KXX_ * KXx;

    // the rest is noise in comparison with the above line.
    for (int i = 0; i < mOutputData.rows(); i++)
    {
        outp(i) = tmp.dot(mOutputData.row(i));
    }
    return outp;
}

template<typename R>
void GaussianProcessRegression<R>::clearTrainingData()
{
    mInputData.resize(mInputData.rows(), 0);
    mOutputData.resize(mOutputData.rows(), 0);
    b_need_prepare_ = true;
    n_data_ = 0;
}

// return the covariance fuction Kff(the covariance between the training data)
template<typename R>
typename GaussianProcessRegression<R>::MatrixXr GaussianProcessRegression<R>::SQEcovFunc(MatrixXr x1)
{
    int nCol = x1.cols();
    MatrixXr retMat(nCol, nCol);
    for (int i = 0; i < nCol; i++)
    {
        for (int j = i; j < nCol; j++)
        {
            retMat(i, j) = SQEcovFuncD(x1.col(i), x1.col(j));
            retMat(j, i) = retMat(i, j);
        }
    }
    return retMat;
}

template<typename R>
void GaussianProcessRegression<R>::debug()
{
    // std::cout << "input data \n" << mInputData << std::endl;
    // std::cout << "output data \n" << mOutputData << std::endl;
    PRINT_DEBUG("input data\n" + mInputData);
    PRINT_DEBUG("output data\n" << mOutputData);
}