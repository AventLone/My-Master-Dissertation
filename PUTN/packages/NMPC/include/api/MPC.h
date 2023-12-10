#pragma once
#include <casadi/casadi.hpp>
#include <Eigen/Dense>

inline Eigen::MatrixXd toEigen(casadi::DM& input)
{
    return Eigen::Map<Eigen::MatrixXd>(input.ptr(), input.rows(), input.columns());
}

class MPC
{
public:
    MPC();
    ~MPC() = default;

    void buildProblem(const Eigen::VectorXd& current_state, const Eigen::MatrixXd& goal_state,
                      const std::vector<Eigen::Vector2d>& obstacle);

    void solve();

    const Eigen::MatrixXd& getStateSolution()
    {
        return mSolution_x;
    }

    const Eigen::MatrixXd& getInputSolution()
    {
        return mSolution_u;
    }

private:
    casadi::Opti mNLP;   // Construct NLP using CasADi

    const int N{10};                                            // MPC Horizon, known as prediction horizon
    const double T{0.2};                                        // Sampling Interval [s]
    const double mMaxVelosity{0.5}, mMaxAngularVelocity{0.6};   // Hard Constraints [m/s, rad/s]
    const float mSafeDistance{0.55f};

    Eigen::MatrixXd mSolution_x;
    Eigen::MatrixXd mSolution_u;

    casadi::MX Q, R;   // Weighting matrices

    casadi::MX mX0;                // Initial condition of state vector
    casadi::MX mUs;                // Control Policy: a sequence of control vectors
    casadi::MX mVelocity;          // Control variable: v [m/s]
    casadi::MX mAngularVelocity;   // Control variable: omega [rad/s]

    /* State variables */
    casadi::MX mXs;           // A sequence of state vectors
    casadi::MX mPosition_x;   // State variable: position.x [m]
    casadi::MX mPosition_y;   // State variable: position.y [m]
    casadi::MX mAngle;        // State variable: theta [rad]
};