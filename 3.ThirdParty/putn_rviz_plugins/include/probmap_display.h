// #pragma once
// #include <rclcpp/rclcpp.hpp>
// #include <nav_msgs/msg/occupancy_grid.hpp>
// #include <nav_msgs/msg/map_meta_data.hpp>
// #include <rviz_common/display.hpp>
// #include <rviz_default_plugins/displays/marker/marker_common.hpp>
// #include <OGRE/OgreTexture.h>
// #include <OGRE/OgreMaterial.h>
// #include <OGRE/OgreVector3.h>

// namespace Ogre
// {
// class ManualObject;
// }

// namespace rviz
// {

// class FloatProperty;
// class IntProperty;
// class Property;
// class QuaternionProperty;
// class RosTopicProperty;
// class VectorProperty;

// /**
//  * \class ProbMapDisplay
//  * \brief Displays a map along the XY plane.
//  */
// class ProbMapDisplay : public rviz_common::Display
// {
//     Q_OBJECT
// public:
//     ProbMapDisplay();
//     virtual ~ProbMapDisplay();

//     // Overrides from Display
//     virtual void onInitialize();
//     virtual void fixedFrameChanged();
//     virtual void reset();
//     virtual void update(float wall_dt, float ros_dt);

//     float getResolution()
//     {
//         return mResolution;
//     }
//     int getWidth()
//     {
//         return mWidth;
//     }
//     int getHeight()
//     {
//         return mHeight;
//     }
//     Ogre::Vector3 getPosition()
//     {
//         return mPosition;
//     }
//     Ogre::Quaternion getOrientation()
//     {
//         return mOrientation;
//     }

// protected Q_SLOTS:
//     void updateAlpha();
//     void updateTopic();
//     void updateDrawUnder();


// protected:
//     // overrides from Display
//     virtual void onEnable();
//     virtual void onDisable();

//     virtual void subscribe();
//     virtual void unsubscribe();

//     void incomingMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg);

//     void clear();

//     void transformMap();

//     Ogre::ManualObject* mManualObject;
//     Ogre::TexturePtr mTexture;
//     Ogre::MaterialPtr mMaterial;
//     bool mLoaded;

//     std::string mTopic;
//     float mResolution;
//     int mWidth;
//     int mHeight;
//     Ogre::Vector3 mPosition;
//     Ogre::Quaternion mOrientation;
//     std::string mFrame;

//     ros::Subscriber mMapSub;

//     RosTopicProperty* mTopicProperty;
//     FloatProperty* mRresolutionProperty;
//     IntProperty* mWidthProperty;
//     IntProperty* mHeightProperty;
//     VectorProperty* mPositionProperty;
//     QuaternionProperty* mOrientationProperty;
//     FloatProperty* mAlphaProperty;
//     Property* mDrawUnderProperty;

//     nav_msgs::msg::OccupancyGrid::ConstSharedPtr mUpdatedMap;
//     nav_msgs::msg::OccupancyGrid::ConstSharedPtr mCurrentMap;
//     std::mutex mMutex;
//     bool mNewMap;
// };

// }   // namespace rviz

// #endif
