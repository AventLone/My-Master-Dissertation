// #include <boost/bind.hpp>

// #include <OGRE/OgreManualObject.h>
// #include <OGRE/OgreMaterialManager.h>
// #include <OGRE/OgreSceneManager.h>
// #include <OGRE/OgreSceneNode.h>
// #include <OGRE/OgreTextureManager.h>

// #include <ros/ros.h>

// #include <tf/transform_listener.h>

// #include "rviz/frame_manager.h"
// #include "rviz/ogre_helpers/grid.h"
// #include "rviz/properties/float_property.h"
// #include "rviz/properties/int_property.h"
// #include "rviz/properties/property.h"
// #include "rviz/properties/quaternion_property.h"
// #include "rviz/properties/ros_topic_property.h"
// #include "rviz/properties/vector_property.h"
// #include "rviz/validate_floats.h"
// #include "rviz/display_context.h"

// #include "multi_probmap_display.h"

// namespace rviz
// {

// MultiProbMapDisplay::MultiProbMapDisplay() : Display(), mLoaded(false), new_map_(false)
// {
//     mTopicProperty = new RosTopicProperty(
//         "Topic",
//         "",
//         QString::fromStdString(ros::message_traits::datatype<multi_map_server::MultiOccupancyGrid>()),
//         "multi_map_server::MultiOccupancyGrid topic to subscribe to.",
//         this,
//         SLOT(updateTopic()));

//     mDrawUnderProperty = new Property("Draw Behind",
//                                         false,
//                                         "Rendering option, controls whether or not the map is always"
//                                         " drawn behind everything else.",
//                                         this,
//                                         SLOT(updateDrawUnder()));
// }

// MultiProbMapDisplay::~MultiProbMapDisplay()
// {
//     unsubscribe();
//     clear();
// }

// void MultiProbMapDisplay::onInitialize()
// {
// }

// void MultiProbMapDisplay::onEnable()
// {
//     subscribe();
// }

// void MultiProbMapDisplay::onDisable()
// {
//     unsubscribe();
//     clear();
// }

// void MultiProbMapDisplay::subscribe()
// {
//     if (!isEnabled())
//     {
//         return;
//     }

//     if (!mTopicProperty->getTopic().isEmpty())
//     {
//         try
//         {
//             mMapSub = update_nh_.subscribe(mTopicProperty->getTopicStd(), 1, &MultiProbMapDisplay::incomingMap, this);
//             setStatus(StatusProperty::Ok, "Topic", "OK");
//         }
//         catch (ros::Exception& e)
//         {
//             setStatus(StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
//         }
//     }
// }

// void MultiProbMapDisplay::unsubscribe()
// {
//     mMapSub.shutdown();
// }

// void MultiProbMapDisplay::updateDrawUnder()
// {
//     bool draw_under = mDrawUnderProperty->getValue().toBool();

//     for (unsigned int k = 0; k < mMaterial.size(); k++) mMaterial[k]->setDepthWriteEnabled(!draw_under);

//     for (unsigned int k = 0; k < mManualObject.size(); k++)
//     {
//         if (draw_under)
//             mManualObject[k]->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
//         else
//             mManualObject[k]->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
//     }
// }

// void MultiProbMapDisplay::updateTopic()
// {
//     unsubscribe();
//     subscribe();
//     clear();
// }

// void MultiProbMapDisplay::clear()
// {
//     setStatus(StatusProperty::Warn, "Message", "No map received");

//     if (!mLoaded)
//     {
//         return;
//     }

//     for (unsigned k = 0; k < mManualObject.size(); k++)
//     {
//         scene_manager_->destroyManualObject(mManualObject[k]);
//         std::string tex_name = mTexture[k]->getName();
//         mTexture[k].setNull();
//         Ogre::TextureManager::getSingleton().unload(tex_name);
//     }
//     mManualObject.clear();
//     mTexture.clear();
//     mMaterial.clear();

//     mLoaded = false;
// }

// // ***********************************************************************************************************************************

// void MultiProbMapDisplay::update(float wall_dt, float ros_dt)
// {
//     {
//         boost::mutex::scoped_lock lock(mMutex);
//         mCurrentMap = mUpdatedMap;
//     }

//     if (!new_map_) return;
//     new_map_ = false;

//     clear();

//     // ros::Time t[5];
//     // double dt[4] = {0,0,0,0};
//     for (unsigned int k = 0; k < mCurrentMap->maps.size(); k++)
//     {
//         if (mCurrentMap->maps[k].data.empty()) continue;
//         setStatus(StatusProperty::Ok, "Message", "Map received");

//         // Map info
//         float resolution = mCurrentMap->maps[k].info.resolution;
//         int width = mCurrentMap->maps[k].info.width;
//         int height = mCurrentMap->maps[k].info.height;

//         // Load pixel
//         // t[0] = ros::Time::now();
//         unsigned int pixels_size = width * height;
//         unsigned char* pixels = new unsigned char[pixels_size];
//         memset(pixels, 255, pixels_size);
//         unsigned int num_pixels_to_copy = pixels_size;
//         if (pixels_size != mCurrentMap->maps[k].data.size())
//             if (mCurrentMap->maps[k].data.size() < pixels_size) num_pixels_to_copy = mCurrentMap->maps[k].data.size();
//         for (unsigned int pixel_index = 0; pixel_index < num_pixels_to_copy; pixel_index++)
//         {
//             unsigned char val;
//             int8_t data = mCurrentMap->maps[k].data[pixel_index];
//             if (data > 0)
//                 val = 255;
//             else if (data < 0)
//                 val = 180;
//             else
//                 val = 0;
//             pixels[pixel_index] = val;
//         }
//         /*
//             int pixels_size = mCurrentMap->maps[k].data.size();
//             unsigned char* pixels = new unsigned char[pixels_size];
//             memcpy(pixels, &mCurrentMap->maps[k].data[0], pixels_size);
//         */
//         // Set texture
//         // t[1] = ros::Time::now();
//         Ogre::DataStreamPtr pixel_stream;
//         pixel_stream.bind(new Ogre::MemoryDataStream(pixels, pixels_size));
//         static int tex_count = 0;
//         std::stringstream ss1;
//         ss1 << "MultiMapTexture" << tex_count++;
//         Ogre::TexturePtr _texture_;
//         // t[2] = ros::Time::now();
//         _texture_ =
//             Ogre::TextureManager::getSingleton().loadRawData(ss1.str(),
//                                                              Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
//                                                              pixel_stream,
//                                                              width,
//                                                              height,
//                                                              Ogre::PF_L8,
//                                                              Ogre::TEX_TYPE_2D,
//                                                              0);
//         // t[3] = ros::Time::now();
//         mTexture.push_back(_texture_);
//         delete[] pixels;
//         setStatus(StatusProperty::Ok, "Map", "Map OK");
//         // t[4] = ros::Time::now();

//         // Set material
//         static int material_count = 0;
//         std::stringstream ss0;
//         ss0 << "MultiMapObjectMaterial" << material_count++;
//         Ogre::MaterialPtr _material_;
//         _material_ = Ogre::MaterialManager::getSingleton().create(
//             ss0.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
//         _material_->setReceiveShadows(false);
//         _material_->getTechnique(0)->setLightingEnabled(false);
//         _material_->setDepthBias(-16.0f, 0.0f);
//         _material_->setCullingMode(Ogre::CULL_NONE);
//         _material_->setDepthWriteEnabled(false);
//         mMaterial.push_back(_material_);
//         mMaterial.back()->setSceneBlending(Ogre::SBT_TRANSPARENT_COLOUR);
//         mMaterial.back()->setDepthWriteEnabled(false);
//         Ogre::Pass* pass = mMaterial.back()->getTechnique(0)->getPass(0);
//         Ogre::TextureUnitState* tex_unit = NULL;
//         if (pass->getNumTextureUnitStates() > 0)
//             tex_unit = pass->getTextureUnitState(0);
//         else
//             tex_unit = pass->createTextureUnitState();
//         tex_unit->setTextureName(mTexture.back()->getName());
//         tex_unit->setTextureFiltering(Ogre::TFO_NONE);

//         // Set manual object
//         static int map_count = 0;
//         std::stringstream ss2;
//         ss2 << "MultiMapObject" << map_count++;
//         Ogre::ManualObject* _manual_object_ = scene_manager_->createManualObject(ss2.str());
//         mManualObject.push_back(_manual_object_);
//         scene_node_->attachObject(mManualObject.back());
//         float yo = tf::getYaw(mCurrentMap->origins[k].orientation);
//         float co = cos(yo);
//         float so = sin(yo);
//         float dxo = mCurrentMap->origins[k].position.x;
//         float dyo = mCurrentMap->origins[k].position.y;
//         float ym = tf::getYaw(mCurrentMap->maps[k].info.origin.orientation);
//         float dxm = mCurrentMap->maps[k].info.origin.position.x;
//         float dym = mCurrentMap->maps[k].info.origin.position.y;
//         float yaw = yo + ym;
//         float c = cos(yaw);
//         float s = sin(yaw);
//         float dx = co * dxm - so * dym + dxo;
//         float dy = so * dxm + co * dym + dyo;
//         float x = 0.0;
//         float y = 0.0;
//         mManualObject.back()->begin(mMaterial.back()->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
//         {
//             // First triangle
//             {
//                 // Bottom left
//                 x = c * 0.0 - s * 0.0 + dx;
//                 y = s * 0.0 + c * 0.0 + dy;
//                 mManualObject.back()->position(x, y, 0.0f);
//                 mManualObject.back()->textureCoord(0.0f, 0.0f);
//                 mManualObject.back()->normal(0.0f, 0.0f, 1.0f);

//                 // Top right
//                 x = c * resolution * width - s * resolution * height + dx;
//                 y = s * resolution * width + c * resolution * height + dy;
//                 mManualObject.back()->position(x, y, 0.0f);
//                 mManualObject.back()->textureCoord(1.0f, 1.0f);
//                 mManualObject.back()->normal(0.0f, 0.0f, 1.0f);

//                 // Top left
//                 x = c * 0.0 - s * resolution * height + dx;
//                 y = s * 0.0 + c * resolution * height + dy;
//                 mManualObject.back()->position(x, y, 0.0f);
//                 mManualObject.back()->textureCoord(0.0f, 1.0f);
//                 mManualObject.back()->normal(0.0f, 0.0f, 1.0f);
//             }

//             // Second triangle
//             {
//                 // Bottom left
//                 x = c * 0.0 - s * 0.0 + dx;
//                 y = s * 0.0 + c * 0.0 + dy;
//                 mManualObject.back()->position(x, y, 0.0f);
//                 mManualObject.back()->textureCoord(0.0f, 0.0f);
//                 mManualObject.back()->normal(0.0f, 0.0f, 1.0f);

//                 // Bottom right
//                 x = c * resolution * width - s * 0.0 + dx;
//                 y = s * resolution * width + c * 0.0 + dy;
//                 mManualObject.back()->position(x, y, 0.0f);
//                 mManualObject.back()->textureCoord(1.0f, 0.0f);
//                 mManualObject.back()->normal(0.0f, 0.0f, 1.0f);

//                 // Top right
//                 x = c * resolution * width - s * resolution * height + dx;
//                 y = s * resolution * width + c * resolution * height + dy;
//                 mManualObject.back()->position(x, y, 0.0f);
//                 mManualObject.back()->textureCoord(1.0f, 1.0f);
//                 mManualObject.back()->normal(0.0f, 0.0f, 1.0f);
//             }
//         }
//         mManualObject.back()->end();
//         if (mDrawUnderProperty->getValue().toBool()) mManualObject.back()->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);

//         // for (int i = 0; i < 4; i++)
//         //   dt[i] += (t[i+1] - t[i]).toSec();
//     }
//     mLoaded = true;
//     context_->queueRender();
//     // ROS_ERROR("RVIZ MAP:  %f %f %f %f", dt[0],dt[1],dt[2],dt[3]);
// }

// // ***********************************************************************************************************************************

// void MultiProbMapDisplay::incomingMap(const multi_map_server::MultiOccupancyGrid::ConstPtr& msg)
// {
//     mUpdatedMap = msg;
//     boost::mutex::scoped_lock lock(mMutex);
//     new_map_ = true;
// }

// void MultiProbMapDisplay::reset()
// {
//     Display::reset();
//     clear();
//     updateTopic();
// }

// }   // namespace rviz

// // #include <pluginlib/class_list_macros.h>
// // PLUGINLIB_EXPORT_CLASS(rviz::MultiProbMapDisplay, rviz::Display)

// #include <pluginlib/class_list_macros.hpp>
// PLUGINLIB_EXPORT_CLASS(rviz::MultiProbMapDisplay, rviz_common::Display)
