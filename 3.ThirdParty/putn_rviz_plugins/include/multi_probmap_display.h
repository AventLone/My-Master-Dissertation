// #pragma once
// #include <OGRE/OgreTexture.h>
// #include <OGRE/OgreMaterial.h>
// #include <OGRE/OgreVector3.h>

// #include <rclcpp/rclcpp.hpp>
// #include <nav_msgs/msg/occupancy_grid.hpp>
// #include <nav_msgs/msg/map_meta_data.hpp>
// #include <rviz_common/display.hpp>
// #include <rviz_default_plugins/displays/marker/marker_common.hpp>
// #include <OGRE/OgreTexture.h>
// #include <OGRE/OgreMaterial.h>
// #include <OGRE/OgreVector3.h>
// #include <multimap.h>

// #include <multi_map_server/MultiOccupancyGrid.h>


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
//  * \class MultiProbMapDisplay
//  * \brief Displays a map along the XY plane.
//  */
// class MultiProbMapDisplay : public rviz_common::Display
// {
//     Q_OBJECT
// public:
//     MultiProbMapDisplay();
//     virtual ~MultiProbMapDisplay();

//     // Overrides from Display
//     virtual void onInitialize();
//     virtual void reset();
//     virtual void update(float wall_dt, float ros_dt);

// protected Q_SLOTS:
//     void updateTopic();
//     void updateDrawUnder();

// protected:
//     // overrides from Display
//     virtual void onEnable();
//     virtual void onDisable();

//     virtual void subscribe();
//     virtual void unsubscribe();

//     void incomingMap(const multi_map_server::MultiOccupancyGrid::ConstPtr& msg);

//     void clear();

//     std::vector<Ogre::ManualObject*> mManualObject;
//     std::vector<Ogre::TexturePtr> mTexture;
//     std::vector<Ogre::MaterialPtr> mMaterial;

//     bool mLoaded;

//     std::string mTopic;

//     ros::Subscriber mMapSub;

//     RosTopicProperty* mTopicProperty;
//     Property* mDrawUnderProperty;

//     multi_map_server::MultiOccupancyGrid::ConstPtr mUpdatedMap;
//     multi_map_server::MultiOccupancyGrid::ConstPtr mCurrentMap;
//     boost::mutex mMutex;
//     bool mNewMap;
// };

// }   // namespace rviz

// #endif
