#include "Controller.h"

static void toRPY(const geometry_msgs::msg::Quaternion& quaternion, double& r, double& p, double& y)
{
    r = std::atan2(2 * (quaternion.w * quaternion.x + quaternion.y + quaternion.z),
                   1 - 2 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y));
    p = std::asin(2 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x));
    y = std::atan2(2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                   1 - 2 * (quaternion.z * quaternion.z + quaternion.y * quaternion.y));
}

Controller::Controller(const std::string& name) : rclcpp::Node(name)
{
    mLocalPlan = Eigen::MatrixXd::Zero(N, 2);

    mTfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);

    mTimer = create_wall_timer(std::chrono::milliseconds(10), std::bind(&Controller::getCurrentState, this));

    mLocalPlanSub = create_subscription<std_msgs::msg::Float32MultiArray>(
        "putn/local_plan",
        rclcpp::ServicesQoS().reliable(),
        std::bind(&Controller::localPlanCallback, this, std::placeholders::_1));
    mVelCmdPub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::ServicesQoS().reliable());
    mCurrStatePub =
        create_publisher<std_msgs::msg::Float32MultiArray>("current_state", rclcpp::ServicesQoS().reliable());
}

void Controller::getCurrentState()
{
    geometry_msgs::msg::TransformStamped transform_msg;

    try
    {
        transform_msg = mTfBuffer->lookupTransform("world", "base_link", tf2::TimePointZero);
    }
    catch (const tf2::TransformException& e)
    {
        return;
    }
    mCurrState(0) = transform_msg.transform.translation.x;
    mCurrState(1) = transform_msg.transform.translation.y;
    mCurrState(2) = transform_msg.transform.translation.z;
    double roll, pitch;
    toRPY(transform_msg.transform.rotation, roll, pitch, mCurrState(3));
    std_msgs::msg::Float32MultiArray state_msg;
    state_msg.data = {static_cast<float>(mCurrState(0)),
                      static_cast<float>(mCurrState(1)),
                      static_cast<float>(mCurrState(2)),
                      static_cast<float>(fmod((mCurrState(3) + M_PI), (2 * M_PI)) - M_PI),
                      static_cast<float>(roll),
                      static_cast<float>(pitch)};
    mCurrStatePub->publish(state_msg);
}

// void Controller::run()
// {
//     // def control_loop(self):
//     //     while not rospy.is_shutdown():
//     //         start_auto = self.manual()
//     //         if(start_auto):
//     //             end_auto = self.auto()
//     //             if not end_auto:
//     //                 break
//     //     self.cmd(np.array([0.0, 0.0]))

//     // def auto(self):
//     //     while not rospy.is_shutdown():
//     //         key = self.getKey()
//     //         if key == 'q':
//     //             return True
//     //         ref_inputs = self.local_plan[0]
//     //         self.cmd(ref_inputs)
//     //         self.rate.sleep()

//     while (rclcpp::ok())
//     {
//         Eigen::Vector2d cmd = mLocalPlan(0, Eigen::all);
//         pubCmd(cmd);
//     }
// }

void Controller::localPlanCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg)
{
    for (int i = 0; i < N; ++i)
    {
        mLocalPlan(i, 0) = msg->data[2 * i];
        mLocalPlan(i, 1) = msg->data[2 * i + 1];
    }

    //----
    Eigen::Vector2d cmd = mLocalPlan(0, Eigen::all);
    pubCmd(cmd);
    //---
}