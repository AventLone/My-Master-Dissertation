// /*
//  * Copyright (c) 2008, Willow Garage, Inc.
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions are met:
//  *
//  *     * Redistributions of source code must retain the above copyright
//  *       notice, this list of conditions and the following disclaimer.
//  *     * Redistributions in binary form must reproduce the above copyright
//  *       notice, this list of conditions and the following disclaimer in the
//  *       documentation and/or other materials provided with the distribution.
//  *     * Neither the name of the Willow Garage, Inc. nor the names of its
//  *       contributors may be used to endorse or promote products derived from
//  *       this software without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  * POSSIBILITY OF SUCH DAMAGE.
//  */

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

// #include "probmap_display.h"

// namespace rviz
// {

// ProbMapDisplay::ProbMapDisplay()
//     : Display(), mManualObject(NULL)
//       // , mMaterial( 0 )
//       ,
//       mLoaded(false), mResolution(0.0f), mWidth(0), mHeight(0), mPosition(Ogre::Vector3::ZERO),
//       mOrientation(Ogre::Quaternion::IDENTITY), new_map_(false)
// {
//     mTopicProperty =
//         new RosTopicProperty("Topic",
//                              "",
//                              QString::fromStdString(ros::message_traits::datatype<nav_msgs::OccupancyGrid>()),
//                              "nav_msgs::OccupancyGrid topic to subscribe to.",
//                              this,
//                              SLOT(updateTopic()));

//     mAlphaProperty =
//         new FloatProperty("Alpha", 0.7, "Amount of transparency to apply to the map.", this, SLOT(updateAlpha()));
//     mAlphaProperty->setMin(0);
//     mAlphaProperty->setMax(1);

//     mDrawUnderProperty = new Property("Draw Behind",
//                                         false,
//                                         "Rendering option, controls whether or not the map is always"
//                                         " drawn behind everything else.",
//                                         this,
//                                         SLOT(updateDrawUnder()));

//     mRresolutionProperty = new FloatProperty("Resolution", 0, "Resolution of the map. (not editable)", this);
//     mRresolutionProperty->setReadOnly(true);

//     mWidthProperty = new IntProperty("Width", 0, "Width of the map, in meters. (not editable)", this);
//     mWidthProperty->setReadOnly(true);

//     mHeightProperty = new IntProperty("Height", 0, "Height of the map, in meters. (not editable)", this);
//     mHeightProperty->setReadOnly(true);

//     mPositionProperty = new VectorProperty("Position",
//                                             Ogre::Vector3::ZERO,
//                                             "Position of the bottom left corner of the map, in meters. (not editable)",
//                                             this);
//     mPositionProperty->setReadOnly(true);

//     mOrientationProperty = new QuaternionProperty(
//         "Orientation", Ogre::Quaternion::IDENTITY, "Orientation of the map. (not editable)", this);
//     mOrientationProperty->setReadOnly(true);
// }

// ProbMapDisplay::~ProbMapDisplay()
// {
//     unsubscribe();
//     clear();
// }

// void ProbMapDisplay::onInitialize()
// {
//     static int count = 0;
//     std::stringstream ss;
//     ss << "MapObjectMaterial" << count++;
//     mMaterial =
//         Ogre::MaterialManager::getSingleton().create(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
//     mMaterial->setReceiveShadows(false);
//     mMaterial->getTechnique(0)->setLightingEnabled(false);
//     mMaterial->setDepthBias(-16.0f, 0.0f);
//     mMaterial->setCullingMode(Ogre::CULL_NONE);
//     mMaterial->setDepthWriteEnabled(false);

//     updateAlpha();
// }

// void ProbMapDisplay::onEnable()
// {
//     subscribe();
// }

// void ProbMapDisplay::onDisable()
// {
//     unsubscribe();
//     clear();
// }

// void ProbMapDisplay::subscribe()
// {
//     if (!isEnabled())
//     {
//         return;
//     }

//     if (!mTopicProperty->getTopic().isEmpty())
//     {
//         try
//         {
//             mMapSub = update_nh_.subscribe(mTopicProperty->getTopicStd(), 1, &ProbMapDisplay::incomingMap, this);
//             setStatus(StatusProperty::Ok, "Topic", "OK");
//         }
//         catch (ros::Exception& e)
//         {
//             setStatus(StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
//         }
//     }
// }

// void ProbMapDisplay::unsubscribe()
// {
//     mMapSub.shutdown();
// }

// void ProbMapDisplay::updateAlpha()
// {
//     float alpha = mAlphaProperty->getFloat();

//     Ogre::Pass* pass = mMaterial->getTechnique(0)->getPass(0);
//     Ogre::TextureUnitState* tex_unit = NULL;
//     if (pass->getNumTextureUnitStates() > 0)
//     {
//         tex_unit = pass->getTextureUnitState(0);
//     }
//     else
//     {
//         tex_unit = pass->createTextureUnitState();
//     }

//     tex_unit->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, alpha);

//     if (alpha < 0.9998)
//     {
//         mMaterial->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
//         mMaterial->setDepthWriteEnabled(false);
//     }
//     else
//     {
//         mMaterial->setSceneBlending(Ogre::SBT_REPLACE);
//         mMaterial->setDepthWriteEnabled(!mDrawUnderProperty->getValue().toBool());
//     }
// }

// void ProbMapDisplay::updateDrawUnder()
// {
//     bool draw_under = mDrawUnderProperty->getValue().toBool();

//     if (mAlphaProperty->getFloat() >= 0.9998)
//     {
//         mMaterial->setDepthWriteEnabled(!draw_under);
//     }

//     if (mManualObject)
//     {
//         if (draw_under)
//         {
//             mManualObject->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
//         }
//         else
//         {
//             mManualObject->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
//         }
//     }
// }

// void ProbMapDisplay::updateTopic()
// {
//     unsubscribe();
//     subscribe();
//     clear();
// }

// void ProbMapDisplay::clear()
// {
//     setStatus(StatusProperty::Warn, "Message", "No map received");

//     if (!mLoaded)
//     {
//         return;
//     }

//     scene_manager_->destroyManualObject(mManualObject);
//     mManualObject = NULL;

//     std::string tex_name = mTexture->getName();
//     mTexture.setNull();
//     Ogre::TextureManager::getSingleton().unload(tex_name);

//     mLoaded = false;
// }

// bool validateFloats(const nav_msgs::OccupancyGrid& msg)
// {
//     bool valid = true;
//     valid = valid && validateFloats(msg.info.resolution);
//     valid = valid && validateFloats(msg.info.origin);
//     return valid;
// }

// void ProbMapDisplay::update(float wall_dt, float ros_dt)
// {
//     {
//         boost::mutex::scoped_lock lock(mMutex);

//         mCurrentMap = mUpdatedMap;
//     }

//     if (!mCurrentMap || !new_map_)
//     {
//         return;
//     }

//     if (mCurrentMap->data.empty())
//     {
//         return;
//     }

//     new_map_ = false;

//     if (!validateFloats(*mCurrentMap))
//     {
//         setStatus(StatusProperty::Error, "Map", "Message contained invalid floating point values (nans or infs)");
//         return;
//     }

//     if (mCurrentMap->info.width * mCurrentMap->info.height == 0)
//     {
//         std::stringstream ss;
//         ss << "Map is zero-sized (" << mCurrentMap->info.width << "x" << mCurrentMap->info.height << ")";
//         setStatus(StatusProperty::Error, "Map", QString::fromStdString(ss.str()));
//         return;
//     }

//     clear();

//     setStatus(StatusProperty::Ok, "Message", "Map received");

//     ROS_DEBUG("Received a %d X %d map @ %.3f m/pix\n",
//               mCurrentMap->info.width,
//               mCurrentMap->info.height,
//               mCurrentMap->info.resolution);

//     float resolution = mCurrentMap->info.resolution;

//     int width = mCurrentMap->info.width;
//     int height = mCurrentMap->info.height;


//     Ogre::Vector3 position(mCurrentMap->info.origin.position.x,
//                            mCurrentMap->info.origin.position.y,
//                            mCurrentMap->info.origin.position.z);
//     Ogre::Quaternion orientation(mCurrentMap->info.origin.orientation.w,
//                                  mCurrentMap->info.origin.orientation.x,
//                                  mCurrentMap->info.origin.orientation.y,
//                                  mCurrentMap->info.origin.orientation.z);
//     mFrame = mCurrentMap->header.frame_id;
//     if (mFrame.empty())
//     {
//         mFrame = "/map";
//     }

//     // Expand it to be RGB data
//     unsigned int pixels_size = width * height;
//     unsigned char* pixels = new unsigned char[pixels_size];
//     memset(pixels, 255, pixels_size);

//     bool map_status_set = false;
//     unsigned int num_pixels_to_copy = pixels_size;
//     if (pixels_size != mCurrentMap->data.size())
//     {
//         std::stringstream ss;
//         ss << "Data size doesn't match width*height: width = " << width << ", height = " << height
//            << ", data size = " << mCurrentMap->data.size();
//         setStatus(StatusProperty::Error, "Map", QString::fromStdString(ss.str()));
//         map_status_set = true;

//         // Keep going, but don't read past the end of the data.
//         if (mCurrentMap->data.size() < pixels_size)
//         {
//             num_pixels_to_copy = mCurrentMap->data.size();
//         }
//     }

//     // TODO: a fragment shader could do this on the video card, and
//     // would allow a non-grayscale color to mark the out-of-range
//     // values.
//     for (unsigned int pixel_index = 0; pixel_index < num_pixels_to_copy; pixel_index++)
//     {
//         unsigned char val;
//         int8_t data = mCurrentMap->data[pixel_index];
//         if (data > 0)
//             val = 0;
//         else if (data < 0)
//             val = 255;
//         else
//             val = 127;
//         pixels[pixel_index] = val;
//     }

//     Ogre::DataStreamPtr pixel_stream;
//     pixel_stream.bind(new Ogre::MemoryDataStream(pixels, pixels_size));
//     static int tex_count = 0;
//     std::stringstream ss;
//     ss << "MapTexture" << tex_count++;
//     try
//     {
//         mTexture =
//             Ogre::TextureManager::getSingleton().loadRawData(ss.str(),
//                                                              Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
//                                                              pixel_stream,
//                                                              width,
//                                                              height,
//                                                              Ogre::PF_L8,
//                                                              Ogre::TEX_TYPE_2D,
//                                                              0);

//         if (!map_status_set)
//         {
//             setStatus(StatusProperty::Ok, "Map", "Map OK");
//         }
//     }
//     catch (Ogre::RenderingAPIException&)
//     {
//         Ogre::Image image;
//         pixel_stream->seek(0);
//         float fwidth = width;
//         float fheight = height;
//         if (width > height)
//         {
//             float aspect = fheight / fwidth;
//             fwidth = 2048;
//             fheight = fwidth * aspect;
//         }
//         else
//         {
//             float aspect = fwidth / fheight;
//             fheight = 2048;
//             fwidth = fheight * aspect;
//         }

//         {
//             std::stringstream ss;
//             ss << "Map is larger than your graphics card supports.  Downsampled from [" << width << "x" << height
//                << "] to [" << fwidth << "x" << fheight << "]";
//             setStatus(StatusProperty::Ok, "Map", QString::fromStdString(ss.str()));
//         }

//         ROS_WARN("Failed to create full-size map texture, likely because your graphics card does not support textures "
//                  "of size > 2048.  Downsampling to [%d x %d]...",
//                  (int)fwidth,
//                  (int)fheight);
//         // ROS_INFO("Stream size [%d], width [%f], height [%f], w * h [%f]", pixel_stream->size(), width, height, width
//         // * height);
//         image.loadRawData(pixel_stream, width, height, Ogre::PF_L8);
//         image.resize(fwidth, fheight, Ogre::Image::FILTER_NEAREST);
//         ss << "Downsampled";
//         mTexture = Ogre::TextureManager::getSingleton().loadImage(
//             ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
//     }

//     delete[] pixels;

//     Ogre::Pass* pass = mMaterial->getTechnique(0)->getPass(0);
//     Ogre::TextureUnitState* tex_unit = NULL;
//     if (pass->getNumTextureUnitStates() > 0)
//     {
//         tex_unit = pass->getTextureUnitState(0);
//     }
//     else
//     {
//         tex_unit = pass->createTextureUnitState();
//     }

//     tex_unit->setTextureName(mTexture->getName());
//     tex_unit->setTextureFiltering(Ogre::TFO_NONE);

//     static int map_count = 0;
//     std::stringstream ss2;
//     ss2 << "MapObject" << map_count++;
//     mManualObject = scene_manager_->createManualObject(ss2.str());
//     scene_node_->attachObject(mManualObject);

//     mManualObject->begin(mMaterial->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
//     {
//         // First triangle
//         {
//             // Bottom left
//             mManualObject->position(0.0f, 0.0f, 0.0f);
//             mManualObject->textureCoord(0.0f, 0.0f);
//             mManualObject->normal(0.0f, 0.0f, 1.0f);

//             // Top right
//             mManualObject->position(resolution * width, resolution * height, 0.0f);
//             mManualObject->textureCoord(1.0f, 1.0f);
//             mManualObject->normal(0.0f, 0.0f, 1.0f);

//             // Top left
//             mManualObject->position(0.0f, resolution * height, 0.0f);
//             mManualObject->textureCoord(0.0f, 1.0f);
//             mManualObject->normal(0.0f, 0.0f, 1.0f);
//         }

//         // Second triangle
//         {
//             // Bottom left
//             mManualObject->position(0.0f, 0.0f, 0.0f);
//             mManualObject->textureCoord(0.0f, 0.0f);
//             mManualObject->normal(0.0f, 0.0f, 1.0f);

//             // Bottom right
//             mManualObject->position(resolution * width, 0.0f, 0.0f);
//             mManualObject->textureCoord(1.0f, 0.0f);
//             mManualObject->normal(0.0f, 0.0f, 1.0f);

//             // Top right
//             mManualObject->position(resolution * width, resolution * height, 0.0f);
//             mManualObject->textureCoord(1.0f, 1.0f);
//             mManualObject->normal(0.0f, 0.0f, 1.0f);
//         }
//     }
//     mManualObject->end();

//     if (mDrawUnderProperty->getValue().toBool())
//     {
//         mManualObject->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
//     }

//     mRresolutionProperty->setValue(resolution);
//     mWidthProperty->setValue(width);
//     mHeightProperty->setValue(height);
//     mPositionProperty->setVector(position);
//     mOrientationProperty->setQuaternion(orientation);

//     transformMap();

//     mLoaded = true;

//     context_->queueRender();
// }

// void ProbMapDisplay::incomingMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
// {

//     mUpdatedMap = msg;
//     boost::mutex::scoped_lock lock(mMutex);
//     new_map_ = true;
// }


// void ProbMapDisplay::transformMap()
// {
//     if (!mCurrentMap)
//     {
//         return;
//     }

//     Ogre::Vector3 position;
//     Ogre::Quaternion orientation;
//     if (!context_->getFrameManager()->transform(mFrame, ros::Time(), mCurrentMap->info.origin, position, orientation))
//     {
//         ROS_DEBUG("Error transforming map '%s' from frame '%s' to frame '%s'",
//                   qPrintable(getName()),
//                   mFrame.c_str(),
//                   qPrintable(fixed_frame_));

//         setStatus(StatusProperty::Error,
//                   "Transform",
//                   "No transform from [" + QString::fromStdString(mFrame) + "] to [" + fixed_frame_ + "]");
//     }
//     else
//     {
//         setStatus(StatusProperty::Ok, "Transform", "Transform OK");
//     }

//     scene_node_->setPosition(position);
//     scene_node_->setOrientation(orientation);
// }

// void ProbMapDisplay::fixedFrameChanged()
// {
//     transformMap();
// }

// void ProbMapDisplay::reset()
// {
//     Display::reset();

//     clear();
//     // Force resubscription so that the map will be re-sent
//     updateTopic();
// }

// }   // namespace rviz

// #include <pluginlib/class_list_macros.h>
// PLUGINLIB_EXPORT_CLASS(rviz::ProbMapDisplay, rviz::Display)
