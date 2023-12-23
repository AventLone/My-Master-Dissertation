#pragma once
#include <QObject>

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

#include "pose_tool.h"
// #endif
#include <rviz_common/properties/string_property.hpp>


// class Arrow;
// class DisplayContext;
// class StringProperty;

class Goal3DTool : public Pose3DTool
{
    Q_OBJECT
public:
    Goal3DTool();
    virtual ~Goal3DTool()
    {
    }
    virtual void onInitialize();
    // virtual void initialize();

protected:
    virtual void onPoseSet(double x, double y, double z, double theta);

private Q_SLOTS:
    void updateTopic();

private:
    // ros::NodeHandle nh_;
    // ros::Publisher pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mGoalPub;

    rviz_common::properties::StringProperty* mTopicProperty;
};
