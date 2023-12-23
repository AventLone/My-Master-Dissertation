#include "RobotDriver.h"
#include <webots/Gyro.hpp>
#include <webots/Accelerometer.hpp>

void RobotDriver::init(webots_ros2_driver::WebotsNode* node, std::unordered_map<std::string, std::string>& parameters)
{
    mNode = node;
    mRobot = std::make_unique<webots::Robot>();

    mLeftMotor = mRobot->getMotor("left track motor");
    mRightMotor = mRobot->getMotor("right track motor");

    mLeftMotor->setPosition(INFINITY);
    mRightMotor->setPosition(INFINITY);

    mLeftMotor->setVelocity(0.0);
    mRightMotor->setVelocity(0.0);

    /*** Imu ***/
    mGyro = mRobot->getGyro("gyro");
    mGyro->enable(mTimeStep);
    mAccelerometer = mRobot->getAccelerometer("accelerometer");
    mAccelerometer->enable(mTimeStep);

    mCmdVelSub = mNode->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        rclcpp::ServicesQoS().reliable(),
        std::bind(&RobotDriver::cmdVelCallback, this, std::placeholders::_1));

    mImuPub = mNode->create_publisher<sensor_msgs::msg::Imu>("BD_Roamer/imu", rclcpp::SensorDataQoS().best_effort());

    /* Frequency: 200 Hz */
    mTimer = mNode->create_wall_timer(std::chrono::milliseconds(5), std::bind(&RobotDriver::publishImu, this));

    // auto f = [this]() -> void
    // {
    //     auto pub = mNode->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::ServicesQoS().reliable());
    //     std::this_thread::sleep_for(std::chrono::seconds(1));
    //     geometry_msgs::msg::Twist msg;
    //     msg.angular.z = M_PI / 2.0;
    //     pub->publish(msg);
    //     std::this_thread::sleep_for(std::chrono::seconds(4));
    //     msg.angular.z = 0.0;
    //     pub->publish(msg);
    // };
    // mThread = std::thread(f);
}

void RobotDriver::cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr& msg)
{
    mCmdVelMsg.linear = msg->linear;
    mCmdVelMsg.angular = msg->angular;
}

void RobotDriver::publishImu()
{
    /*** Publish imu data ***/
    auto angular_vel = mGyro->getValues();
    auto accele = mAccelerometer->getValues();
    mImuMsg.angular_velocity.x = angular_vel[0];
    mImuMsg.angular_velocity.y = angular_vel[1];
    mImuMsg.angular_velocity.z = angular_vel[2];
    mImuMsg.linear_acceleration.x = accele[0];
    mImuMsg.linear_acceleration.y = accele[1];
    mImuMsg.linear_acceleration.z = accele[2];
    mImuMsg.header.stamp = rclcpp::Clock().now();
    mImuPub->publish(mImuMsg);
}

void RobotDriver::step()
{
    auto linear_speed = mCmdVelMsg.linear.x;
    auto angular_speed = mCmdVelMsg.angular.z;

    auto command_motor_left = (linear_speed - angular_speed * mHalfDistanceBetweenWheels) / mWheelRadius;
    auto command_motor_right = (linear_speed + angular_speed * mHalfDistanceBetweenWheels) / mWheelRadius;


    // auto command_motor_left = -mCmdVelMsg.linear.x;
    // auto command_motor_right = mCmdVelMsg.linear.x;

    // auto command_motor_left = linear_speed - 4.98755 * angular_speed;
    // auto command_motor_right = linear_speed + 4.98755 * angular_speed;

    mLeftMotor->setVelocity(command_motor_left);   // set angular velocity
    mRightMotor->setVelocity(command_motor_right);
}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(RobotDriver, webots_ros2_driver::PluginInterface)