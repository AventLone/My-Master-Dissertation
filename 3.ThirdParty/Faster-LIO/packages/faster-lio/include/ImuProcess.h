#pragma once
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cmath>
#include <deque>
#include <fstream>

#include "common_lib.hpp"
#include "so3_math.hpp"
#include "use-ikfom.h"

inline double toSec(const builtin_interfaces::msg::Time& time)
{
    return static_cast<double>(time.sec) + static_cast<double>(time.nanosec) * 1e-9;
}

namespace faster_lio
{
constexpr int MAX_INI_COUNT = 20;

inline bool time_list(const PointType& x, const PointType& y)
{
    return (x.curvature < y.curvature);
}

/* IMU process and undistortion */
class ImuProcess
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();
    ~ImuProcess() = default;

    void reset();

    void setExtrinsic(const common::V3D& transl, const common::M3D& rot)
    {
        Lidar_T_wrt_IMU_ = transl;
        Lidar_R_wrt_IMU_ = rot;
    }

    void setGyrCov(const common::V3D& scaler)
    {
        cov_gyr_scale_ = scaler;
    }

    void setAccCov(const common::V3D& scaler)
    {
        cov_acc_scale_ = scaler;
    }

    void setGyrBiasCov(const common::V3D& b_g)
    {
        cov_bias_gyr_ = b_g;
    }

    void setAccBiasCov(const common::V3D& b_a)
    {
        cov_bias_acc_ = b_a;
    }

    void process(const common::MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state,
                 PointCloudType::Ptr pcl_un_);

    std::ofstream fout_imu_;
    Eigen::Matrix<double, 12, 12> Q_;
    common::V3D cov_acc_;
    common::V3D cov_gyr_;
    common::V3D cov_acc_scale_;
    common::V3D cov_gyr_scale_;
    common::V3D cov_bias_gyr_;
    common::V3D cov_bias_acc_;

private:
    void IMUInit(const common::MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state, int& N);
    void undistortPcl(const common::MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state,
                      PointCloudType& pcl_out);

    PointCloudType::Ptr cur_pcl_un_;
    sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> v_imu_;
    std::vector<common::Pose6Dmsg> IMUpose_;
    std::vector<common::M3D> v_rot_pcl_;
    common::M3D Lidar_R_wrt_IMU_;
    common::V3D Lidar_T_wrt_IMU_;
    common::V3D mean_acc_;
    common::V3D mean_gyr_;
    common::V3D angvel_last_;
    common::V3D acc_s_last_;
    double last_lidar_end_time_ = 0;
    int init_iter_num_ = 1;
    bool b_first_frame_ = true;
    bool imu_need_init_ = true;
};
}   // namespace faster_lio