#pragma once
#include <Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class Controller : public rclcpp::Node
{
public:
    explicit Controller(const std::string& name);
    ~Controller() = default;

private:
    /* Parameters */
    // const int N{10};   // Prediction Horizon
    static constexpr int N{10};   // Prediction Horizon

    /* Buffers */
    Eigen::Vector4d mCurrState;
    // Eigen::MatrixXd mLocalPlan;
    Eigen::Matrix<double, N, 2> mLocalPlan;

    geometry_msgs::msg::Twist mControlCmdMsg;

    std::unique_ptr<tf2_ros::Buffer> mTfBuffer{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> mTfListener{nullptr};

    /* Subscriptions */
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr mLocalPlanSub;

    /* Publishers */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mVelCmdPub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mCurrStatePub;
    rclcpp::TimerBase::SharedPtr mTimer;

    // def get_current_state(self, event):
    //         try:
    //             (trans, rot) = self.listener.lookupTransform(
    //                 'world', 'base_link', rospy.Time(0))

    //             self.curr_state[0] = trans[0]
    //             self.curr_state[1] = trans[1]
    //             self.curr_state[2] = trans[2]
    //             roll, pitch, self.curr_state[3] = self.quart_to_rpy(
    //                 rot[0], rot[1], rot[2], rot[3])  # r,p,y
    //             c = Float32MultiArray()
    //             c.data = [self.curr_state[0], self.curr_state[1], self.curr_state[2],
    //                       (self.curr_state[3]+np.pi) % (2*np.pi)-np.pi, roll, pitch]
    //             self.pub2.publish(c)
    //         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    //             pass
    void getCurrentState();

    void pubCmd(const Eigen::Vector2d& cmd)
    {
        mControlCmdMsg.linear.x = cmd(0);
        mControlCmdMsg.angular.z = cmd(1);
        mVelCmdPub->publish(mControlCmdMsg);
    }

    void run();

    void localPlanCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg);
};