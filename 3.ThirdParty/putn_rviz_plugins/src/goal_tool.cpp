#include "goal_tool.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rviz_common/display_context.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <rviz_common/properties/string_property.hpp>


Goal3DTool::Goal3DTool()
{
    shortcut_key_ = 'g';
    mTopicProperty = new rviz_common::properties::StringProperty("Topic",
                                                                 "goal",
                                                                 "The topic on which to publish navigation goals.",
                                                                 getPropertyContainer(),
                                                                 SLOT(updateTopic()),
                                                                 this);
}

void Goal3DTool::onInitialize()
{
    Pose3DTool::onInitialize();
    setName("3D Nav Goal");
    updateTopic();
}

void Goal3DTool::updateTopic()
{
    // pub_ = nh_.advertise<geometry_msgs::msg::PoseStamped>(mTopicProperty->getStdString(), 1);
    mGoalPub = context_->getRosNodeAbstraction().lock()->create_publisher<geometry_msgs::msg::PoseStamped>(
        mTopicProperty->getStdString(), 1);
}

void Goal3DTool::onPoseSet(double x, double y, double z, double theta)
{
    // ROS_WARN("3D Goal Set");
    // std::string fixed_frame = context_->getFixedFrame().toStdString();
    // tf2::Quaternion quat;
    // quat.setRPY(0.0, 0.0, theta);

    // tf2::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, z)), ros::Time::now(),
    // fixed_frame); geometry_msgs::msg::PoseStamped goal; tf2::poseStampedTFToMsg(p, goal);

    std::string fixed_frame = context_->getFixedFrame().toStdString();
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, theta);

    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(x, y, z));
    transform.setRotation(quat);

    geometry_msgs::msg::PoseStamped goal;
    tf2::Stamped<tf2::Transform> stampedTransform(transform, tf2::TimePoint(), fixed_frame);
    tf2::toMsg(stampedTransform, goal.pose);
    goal.header.stamp = rclcpp::Clock().now();
    goal.header.frame_id = fixed_frame;

    // ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle:
    // %.3f\n",
    //          fixed_frame.c_str(),
    //          goal.pose.position.x,
    //          goal.pose.position.y,
    //          goal.pose.position.z,
    //          goal.pose.orientation.x,
    //          goal.pose.orientation.y,
    //          goal.pose.orientation.z,
    //          goal.pose.orientation.w,
    //          theta);
    // pub_.publish(goal);

    mGoalPub->publish(goal);
}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(Goal3DTool, rviz_common::Tool)
