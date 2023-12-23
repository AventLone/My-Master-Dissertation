#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <rviz_common/display.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreVector3.h>

// namespace Ogre
// {
// class ManualObject;
// }


// class FloatProperty;
// class IntProperty;
// class Property;
// class QuaternionProperty;
// class RosTopicProperty;
// class VectorProperty;

/**
 * \class AerialMapDisplay
 * \brief Displays a map along the XY plane.
 */
class AerialMapDisplay : public rviz_common::Display
{
    Q_OBJECT
public:
    AerialMapDisplay();
    virtual ~AerialMapDisplay();

    // Overrides from Display
    virtual void onInitialize();
    virtual void fixedFrameChanged();
    virtual void reset();
    virtual void update(float wall_dt, float ros_dt);

    float getResolution()
    {
        return mResolution;
    }
    int getWidth()
    {
        return mWidth;
    }
    int getHeight()
    {
        return mHeight;
    }
    Ogre::Vector3 getPosition()
    {
        return mPosition;
    }
    Ogre::Quaternion getOrientation()
    {
        return mOrientation;
    }

protected Q_SLOTS:
    void updateAlpha();
    void updateTopic();
    void updateDrawUnder();


protected:
    // overrides from Display
    virtual void onEnable();
    virtual void onDisable();

    virtual void subscribe();
    virtual void unsubscribe();

    void incomingAerialMap(const nav_msgs::msg::OccupancyGrid::ConstPtr& msg);

    void clear();

    void transformAerialMap();

    Ogre::ManualObject* mManualObject;
    Ogre::TexturePtr mTexture;
    Ogre::MaterialPtr mMaterial;
    bool mLoaded;

    std::string mTopic;
    float mResolution;
    int mWidth;
    int mHeight;
    Ogre::Vector3 mPosition;
    Ogre::Quaternion mOrientation;
    std::string mFrame;

    // ros::Subscriber mMapSub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mMapSub;

    rviz_common::properties::RosTopicProperty* mTopicProperty;
    rviz_common::properties::FloatProperty* mRresolutionProperty;
    rviz_common::properties::IntProperty* mWidthProperty;
    rviz_common::properties::IntProperty* mHeightProperty;
    rviz_common::properties::VectorProperty* mPositionProperty;
    rviz_common::properties::QuaternionProperty* mOrientationProperty;
    rviz_common::properties::FloatProperty* mAlphaProperty;
    rviz_common::properties::Property* mDrawUnderProperty;

    nav_msgs::msg::OccupancyGrid::ConstPtr mUpdatedMap;
    nav_msgs::msg::OccupancyGrid::ConstPtr mCurrentMap;
    std::mutex mMutex;
    bool mNewMap;
};
