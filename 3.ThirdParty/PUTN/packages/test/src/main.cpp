#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto test_node = std::make_shared<rclcpp::Node>("test_node");

    auto vel_pub =
        test_node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::ParametersQoS().reliable());
    auto goal_pub =
        test_node->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", rclcpp::ServicesQoS().reliable());

    auto mTfBuffer = std::make_unique<tf2_ros::Buffer>(test_node->get_clock());
    auto mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);

    geometry_msgs::msg::Twist twist_msg;

    auto moveForward = [&](float distance) -> void   // unit:m
    {
        twist_msg.angular.z = 0.0;
        twist_msg.linear.x = distance > 0.0f ? 0.2 : -0.2;
        vel_pub->publish(twist_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(distance * 1000.0f / 0.2f)));
    };
    auto rotate = [&](float rad) -> void
    {
        twist_msg.angular.z = rad > 0.0f ? (M_PI / 4.0) : (-M_PI / 4.0);
        twist_msg.linear.x = 0.0;
        vel_pub->publish(twist_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(rad * 1000.0f * 4.0f / M_PI)));
    };
    auto stop = [&]() -> void
    {
        twist_msg.angular.z = 0.0;
        twist_msg.linear.x = 0.0;
        vel_pub->publish(twist_msg);
    };

    moveForward(2.7f);
    stop();
    rclcpp::sleep_for(std::chrono::seconds(3));

    // 8.342577, 0.922656, -0.751795
    // std::thread thread_1(
    //     [&]()
    //     {

    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.pose.position.x = 8.342577;
    goal_msg.pose.position.y = 0.822656;
    // goal_msg.header.stamp = test_node->now();
    goal_pub->publish(goal_msg);

    // });

    // moveForward(1.5f);
    // twist_msg.angular.z = 0.0;
    // twist_msg.linear.x = 0.2;
    // vel_pub->publish(twist_msg);
    // rclcpp::sleep_for(std::chrono::milliseconds(6000));
    // moveForward(0.5f);
    twist_msg.angular.z = -M_PI / 6.0;
    twist_msg.linear.x = 0.2;
    vel_pub->publish(twist_msg);
    rclcpp::sleep_for(std::chrono::seconds(7));

    // moveForward(1.8f);
    twist_msg.angular.z = M_PI / 6.0;
    twist_msg.linear.x = 0.2;
    vel_pub->publish(twist_msg);
    rclcpp::sleep_for(std::chrono::seconds(13));
    moveForward(3.0f);
    // moveForward(2.0f);
    stop();

    geometry_msgs::msg::TransformStamped transform_msg;
    try
    {
        transform_msg = mTfBuffer->lookupTransform("world", "base_link", tf2::TimePointZero);
        RCLCPP_INFO(test_node->get_logger(),
                    "Terminal: %f, %f, %f",
                    transform_msg.transform.translation.x,
                    transform_msg.transform.translation.y,
                    transform_msg.transform.translation.z);
    }
    catch (const tf2::TransformException& e)
    {
        RCLCPP_ERROR(test_node->get_logger(), e.what());
    }
    // thread_1.join();
    return 0;
}