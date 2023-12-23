#include "pose_tool.h"
#include <OGRE/OgrePlane.h>
#include <OGRE/OgreRay.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreViewport.h>
#include <rviz_rendering/geometry.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/load_resource.hpp>
// #include <rviz_rendering/objects/arrow.hpp>

Pose3DTool::Pose3DTool() : rviz_common::Tool(), mAarrow(nullptr)
{
}

Pose3DTool::~Pose3DTool()
{
    // delete mAarrow;
}

void Pose3DTool::onInitialize()
{
    mAarrow = std::make_shared<rviz_rendering::Arrow>(scene_manager_, nullptr, 2.0f, 0.2f, 0.5f, 0.35f);
    mAarrow->setColor(0.0f, 1.0f, 0.0f, 1.0f);
    mAarrow->getSceneNode()->setVisible(false);
}

void Pose3DTool::activate()
{
    setStatus("Click and drag mouse to set position/orientation.");
    mState = Position;
}

void Pose3DTool::deactivate()
{
    mAarrow->getSceneNode()->setVisible(false);
}

int Pose3DTool::processMouseEvent(rviz_common::ViewportMouseEvent& event)
{
    int flags = 0;
    static Ogre::Vector3 ang_pos;
    static double initz;
    static double prevz;
    static double prevangle;
    const double z_scale = 50;
    const double z_interval = 0.5;
    Ogre::Quaternion orient_x = Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y);

    auto point_projection_on_xy_plane =
        mProjectionFinder->getViewportPointProjectionOnXYPlane(event.panel->getRenderWindow(), event.x, event.y);

    if (event.leftDown())
    {
        // ROS_ASSERT(mState == Position);
        assert(mState == Position);

        Ogre::Vector3 intersection;
        Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
        if (getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x, event.y, intersection))
        {
            mArrowPosition = intersection;
            mAarrow->setPosition(mArrowPosition);
            mState = Orientation;
            flags |= Render;
        }
    }
    else if (event.type == QEvent::MouseMove && event.left())
    {
        if (mState == Orientation)
        {
            // compute angle in x-y plane
            Ogre::Vector3 cur_pos;
            Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);

            if (getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x, event.y, cur_pos))
            {
                double angle = atan2(cur_pos.y - mArrowPosition.y, cur_pos.x - mArrowPosition.x);
                mAarrow->getSceneNode()->setVisible(true);
                mAarrow->setOrientation(Ogre::Quaternion(Ogre::Radian(angle), Ogre::Vector3::UNIT_Z) * orient_x);
                if (event.right()) mState = Height;
                initz = mArrowPosition.z;
                prevz = event.y;
                prevangle = angle;
                flags |= Render;
            }
        }
        if (mState == Height)
        {
            double z = event.y;
            double dz = z - prevz;
            prevz = z;
            mArrowPosition.z -= dz / z_scale;
            mAarrow->setPosition(mArrowPosition);
            // Create a list of arrows
            // for (int k = 0; k < mArrowArray.size(); k++) delete mArrowArray[k];
            mArrowArray.clear();
            int cnt = ceil(fabs(initz - mArrowPosition.z) / z_interval);
            for (int k = 0; k < cnt; k++)
            {
                rviz_rendering::Arrow* arrow;
                arrow = std::make_shared<rviz_rendering::Arrow>(scene_manager_, nullptr, 0.5f, 0.1f, 0.0f, 0.1f);
                arrow->setColor(0.0f, 1.0f, 0.0f, 1.0f);
                arrow->getSceneNode()->setVisible(true);
                Ogre::Vector3 arr_pos = mArrowPosition;
                arr_pos.z = initz - ((initz - mArrowPosition.z > 0) ? 1 : -1) * k * z_interval;
                arrow->setPosition(arr_pos);
                arrow->setOrientation(Ogre::Quaternion(Ogre::Radian(prevangle), Ogre::Vector3::UNIT_Z) * orient_x);
                mArrowArray.push_back(arrow);
            }
            flags |= Render;
        }
    }
    else if (event.leftUp())
    {
        if (mState == Orientation || mState == Height)
        {
            // Create a list of arrows
            // for (int k = 0; k < mArrowArray.size(); k++) delete mArrowArray[k];
            mArrowArray.clear();
            onPoseSet(mArrowPosition.x, mArrowPosition.y, mArrowPosition.z, prevangle);
            flags |= (Finished | Render);
        }
    }

    return flags;
}
