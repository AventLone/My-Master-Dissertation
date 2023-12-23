#pragma once
// #include <OGRE/OgreVector3.h>
// #include <QCursor>
// #include <rclcpp/rclcpp.hpp>
// #include <rviz_common/tool.hpp>


#include <memory>
#include <string>
#include <utility>

#include <OgreVector3.h>

#include <QCursor>   // NOLINT cpplint cannot handle include order here

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <rviz_common/tool.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>
#include <rviz_rendering/objects/arrow.hpp>

// class Arrow;
// class DisplayContext;

// class Pose3DTool : public rviz_common::Tool
// {
// public:
//     Pose3DTool();
//     virtual ~Pose3DTool();

//     virtual void onInitialize();

//     virtual void activate();
//     virtual void deactivate();

//     virtual int processMouseEvent(rviz_common::ViewportMouseEvent& event);

// protected:
//     virtual void onPoseSet(double x, double y, double z, double theta) = 0;

//     rviz_rendering::Arrow* mAarrow;
//     std::vector<rviz_rendering::Arrow*> mArrowArray;

//     enum State
//     {
//         Position,
//         Orientation,
//         Height
//     };
//     State mState;

//     Ogre::Vector3 mPos;
// };

class Pose3DTool : public rviz_common::Tool
{
public:
    Pose3DTool();
    ~Pose3DTool() override;

    void onInitialize() override;

    void activate() override;

    void deactivate() override;

    int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

protected:
    virtual void onPoseSet(double x, double y, double z, double theta) = 0;

    std::shared_ptr<rviz_rendering::Arrow> mAarrow;
    std::vector<std::shared_ptr<rviz_rendering::Arrow>> mArrowArray;

    enum State
    {
        Position,
        Orientation,
        Height
    };
    State mState;

    Ogre::Vector3 mArrowPosition;

    std::shared_ptr<rviz_rendering::ViewportProjectionFinder> mProjectionFinder;
};
