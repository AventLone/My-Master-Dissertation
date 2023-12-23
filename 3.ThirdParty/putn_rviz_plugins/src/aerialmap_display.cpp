#include "aerialmap_display.h"
// #include <boost/bind.hpp>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTextureManager.h>
#include <tf2_ros/transform_listener.h>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/quaternion_property.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_common/display_context.hpp>

// #include "rviz/frame_manager.h"
// #include "rviz/ogre_helpers/grid.h"


namespace rviz
{

AerialMapDisplay::AerialMapDisplay()
    : rviz_common::Display(), mManualObject(nullptr)
      //, mMaterial( 0 )
      ,
      mLoaded(false), mResolution(0.0f), mWidth(0), mHeight(0), mPosition(Ogre::Vector3::ZERO),
      mOrientation(Ogre::Quaternion::IDENTITY), mNewMap(false)
{
    mTopicProperty = new rviz_common::properties::RosTopicProperty(
        "Topic",
        "",
        QString::fromStdString(ros::message_traits::datatype<nav_msgs::OccupancyGrid>()),
        "nav_msgs::OccupancyGrid topic to subscribe to.",
        this,
        SLOT(updateTopic()));

    mAlphaProperty = new rviz_common::properties::FloatProperty(
        "Alpha", 0.7, "Amount of transparency to apply to the map.", this, SLOT(updateAlpha()));
    mAlphaProperty->setMin(0);
    mAlphaProperty->setMax(1);

    mDrawUnderProperty = new Property("Draw Behind",
                                      false,
                                      "Rendering option, controls whether or not the map is always"
                                      " drawn behind everything else.",
                                      this,
                                      SLOT(updateDrawUnder()));

    mRresolutionProperty =
        new rviz_common::properties::FloatProperty("Resolution", 0, "Resolution of the map. (not editable)", this);
    mRresolutionProperty->setReadOnly(true);

    mWidthProperty =
        new rviz_common::properties::IntProperty("Width", 0, "Width of the map, in meters. (not editable)", this);
    mWidthProperty->setReadOnly(true);

    mHeightProperty =
        new rviz_common::properties::IntProperty("Height", 0, "Height of the map, in meters. (not editable)", this);
    mHeightProperty->setReadOnly(true);

    mPositionProperty = new rviz_common::properties::VectorProperty(
        "Position",
        Ogre::Vector3::ZERO,
        "Position of the bottom left corner of the map, in meters. (not editable)",
        this);
    mPositionProperty->setReadOnly(true);

    mOrientationProperty = new rviz_common::properties::QuaternionProperty(
        "Orientation", Ogre::Quaternion::IDENTITY, "Orientation of the map. (not editable)", this);
    mOrientationProperty->setReadOnly(true);
}

AerialMapDisplay::~AerialMapDisplay()
{
    unsubscribe();
    clear();
}

void AerialMapDisplay::onInitialize()
{
    static int count = 0;
    std::stringstream ss;
    ss << "AerialMapObjectMaterial" << count++;
    mMaterial =
        Ogre::MaterialManager::getSingleton().create(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    mMaterial->setReceiveShadows(false);
    mMaterial->getTechnique(0)->setLightingEnabled(false);
    mMaterial->setDepthBias(-16.0f, 0.0f);
    mMaterial->setCullingMode(Ogre::CULL_NONE);
    mMaterial->setDepthWriteEnabled(false);

    updateAlpha();
}

void AerialMapDisplay::onEnable()
{
    subscribe();
}

void AerialMapDisplay::onDisable()
{
    unsubscribe();
    clear();
}

// void AerialMapDisplay::subscribe()
// {
//     if (!isEnabled())
//     {
//         return;
//     }

//     if (!mTopicProperty->getTopic().isEmpty())
//     {
//         try
//         {
//             mMapSub =
//                 update_nh_.subscribe(mTopicProperty->getTopicStd(), 1, &AerialMapDisplay::incomingAerialMap, this);
//             setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
//         }
//         catch (const std::exception& e)
//         {
//             setStatus(
//                 rviz_common::properties::StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
//         }
//     }
// }

void AerialMapDisplay::subscribe()
{
    if (!isEnabled())
    {
        return;
    }

    if (topic_property_->isEmpty())
    {
        setStatus(properties::StatusProperty::Error, "Topic", QString("Error subscribing: Empty topic name"));
        return;
    }

    try
    {
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.event_callbacks.message_lost_callback = [&](rclcpp::QOSMessageLostInfo& info)
        {
            std::ostringstream sstm;
            sstm << "Some messages were lost:\n>\tNumber of new lost messages: " << info.total_count_change
                 << " \n>\tTotal number of messages lost: " << info.total_count;
            setStatus(properties::StatusProperty::Warn, "Topic", QString(sstm.str().c_str()));
        };

        // TODO(anhosi,wjwwood): replace with abstraction for subscriptions once available
        subscription_ = rviz_ros_node_.lock()->get_raw_node()->template create_subscription<MessageType>(
            topic_property_->getTopicStd(),
            qos_profile,
            [this](const typename MessageType::ConstSharedPtr message) { incomingMessage(message); },
            sub_opts);
        setStatus(properties::StatusProperty::Ok, "Topic", "OK");
    }
    catch (rclcpp::exceptions::InvalidTopicNameError& e)
    {
        setStatus(properties::StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
    }
}

void AerialMapDisplay::unsubscribe()
{
    mMapSub.shutdown();
}

void AerialMapDisplay::updateAlpha()
{
    float alpha = mAlphaProperty->getFloat();

    Ogre::Pass* pass = mMaterial->getTechnique(0)->getPass(0);
    Ogre::TextureUnitState* tex_unit = NULL;
    if (pass->getNumTextureUnitStates() > 0)
    {
        tex_unit = pass->getTextureUnitState(0);
    }
    else
    {
        tex_unit = pass->createTextureUnitState();
    }

    tex_unit->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, alpha);

    if (alpha < 0.9998)
    {
        mMaterial->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        mMaterial->setDepthWriteEnabled(false);
    }
    else
    {
        mMaterial->setSceneBlending(Ogre::SBT_REPLACE);
        mMaterial->setDepthWriteEnabled(!mDrawUnderProperty->getValue().toBool());
    }
}

void AerialMapDisplay::updateDrawUnder()
{
    bool draw_under = mDrawUnderProperty->getValue().toBool();

    if (mAlphaProperty->getFloat() >= 0.9998)
    {
        mMaterial->setDepthWriteEnabled(!draw_under);
    }

    if (mManualObject)
    {
        if (draw_under)
        {
            mManualObject->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
        }
        else
        {
            mManualObject->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
        }
    }
}

void AerialMapDisplay::updateTopic()
{
    unsubscribe();
    subscribe();
    clear();
}

void AerialMapDisplay::clear()
{
    setStatus(rviz_common::properties::StatusProperty::Warn, "Message", "No map received");

    if (!mLoaded)
    {
        return;
    }

    scene_manager_->destroyManualObject(mManualObject);
    mManualObject = NULL;

    std::string tex_name = mTexture->getName();
    mTexture.setNull();
    Ogre::TextureManager::getSingleton().unload(tex_name);

    mLoaded = false;
}

void AerialMapDisplay::update(float wall_dt, float ros_dt)
{
    {
        // boost::mutex::scoped_lock lock(mMutex);
        std::unique_lock<std::mutex> lock(mMutex);

        mCurrentMap = mUpdatedMap;
    }

    if (!mCurrentMap || !mNewMap)
    {
        return;
    }

    if (mCurrentMap->data.empty())
    {
        return;
    }

    mNewMap = false;

    if (mCurrentMap->info.width * mCurrentMap->info.height == 0)
    {
        std::stringstream ss;
        ss << "AerialMap is zero-sized (" << mCurrentMap->info.width << "x" << mCurrentMap->info.height << ")";
        setStatus(rviz_common::properties::StatusProperty::Error, "AerialMap", QString::fromStdString(ss.str()));
        return;
    }

    clear();

    setStatus(rviz_common::properties::StatusProperty::Ok, "Message", "AerialMap received");

    // ROS_DEBUG("Received a %d X %d map @ %.3f m/pix\n",
    //           mCurrentMap->info.width,
    //           mCurrentMap->info.height,
    //           mCurrentMap->info.resolution);

    float resolution = mCurrentMap->info.resolution;

    int width = mCurrentMap->info.width;
    int height = mCurrentMap->info.height;


    Ogre::Vector3 position(
        mCurrentMap->info.origin.position.x, mCurrentMap->info.origin.position.y, mCurrentMap->info.origin.position.z);
    Ogre::Quaternion orientation(mCurrentMap->info.origin.orientation.w,
                                 mCurrentMap->info.origin.orientation.x,
                                 mCurrentMap->info.origin.orientation.y,
                                 mCurrentMap->info.origin.orientation.z);
    mFrame = mCurrentMap->header.frame_id;
    if (mFrame.empty())
    {
        mFrame = "map";
    }

    // Expand it to be RGB data
    unsigned int pixels_size = width * height * 3;
    unsigned char* pixels = new unsigned char[pixels_size];
    std::memset(pixels, 255, pixels_size);

    bool map_status_set = false;
    unsigned int num_pixels_to_copy = pixels_size;
    if (pixels_size != mCurrentMap->data.size())
    {
        std::stringstream ss;
        ss << "Data size doesn't match width*height: width = " << width << ", height = " << height
           << ", data size = " << mCurrentMap->data.size();
        setStatus(rviz_common::properties::StatusProperty::Error, "AerialMap", QString::fromStdString(ss.str()));
        map_status_set = true;

        // Keep going, but don't read past the end of the data.
        if (mCurrentMap->data.size() < pixels_size)
        {
            num_pixels_to_copy = mCurrentMap->data.size();
        }
    }

    // TODO: a fragment shader could do this on the video card, and
    // would allow a non-grayscale color to mark the out-of-range
    // values.
    for (unsigned int pixel_index = 0; pixel_index < num_pixels_to_copy; pixel_index++)
    {
        pixels[pixel_index] = mCurrentMap->data[pixel_index];
    }

    Ogre::DataStreamPtr pixel_stream;
    pixel_stream.bind(new Ogre::MemoryDataStream(pixels, pixels_size));
    static int tex_count = 0;
    std::stringstream ss;
    ss << "AerialMapTexture" << tex_count++;
    try
    {
        mTexture =
            Ogre::TextureManager::getSingleton().loadRawData(ss.str(),
                                                             Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                             pixel_stream,
                                                             width,
                                                             height,
                                                             Ogre::PF_R8G8B8,
                                                             Ogre::TEX_TYPE_2D,
                                                             0);

        if (!map_status_set)
        {
            setStatus(rviz_common::properties::StatusProperty::Ok, "AerialMap", "AerialMap OK");
        }
    }
    catch (Ogre::RenderingAPIException&)
    {
        Ogre::Image image;
        pixel_stream->seek(0);
        float fwidth = width;
        float fheight = height;
        if (width > height)
        {
            float aspect = fheight / fwidth;
            fwidth = 2048;
            fheight = fwidth * aspect;
        }
        else
        {
            float aspect = fwidth / fheight;
            fheight = 2048;
            fwidth = fheight * aspect;
        }

        {
            std::stringstream ss;
            ss << "AerialMap is larger than your graphics card supports.  Downsampled from [" << width << "x" << height
               << "] to [" << fwidth << "x" << fheight << "]";
            setStatus(rviz_common::properties::StatusProperty::Ok, "AerialMap", QString::fromStdString(ss.str()));
        }

        // ROS_WARN("Failed to create full-size map texture, likely because your graphics card does not support textures
        // "
        //          "of size > 2048.  Downsampling to [%d x %d]...",
        //          (int)fwidth,
        //          (int)fheight);

        image.loadRawData(pixel_stream, width, height, Ogre::PF_R8G8B8);
        image.resize(fwidth, fheight, Ogre::Image::FILTER_NEAREST);
        ss << "Downsampled";
        mTexture = Ogre::TextureManager::getSingleton().loadImage(
            ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
    }

    delete[] pixels;

    Ogre::Pass* pass = mMaterial->getTechnique(0)->getPass(0);
    Ogre::TextureUnitState* tex_unit = NULL;
    if (pass->getNumTextureUnitStates() > 0)
    {
        tex_unit = pass->getTextureUnitState(0);
    }
    else
    {
        tex_unit = pass->createTextureUnitState();
    }

    tex_unit->setTextureName(mTexture->getName());
    tex_unit->setTextureFiltering(Ogre::TFO_NONE);

    static int map_count = 0;
    std::stringstream ss2;
    ss2 << "AerialMapObject" << map_count++;
    mManualObject = scene_manager_->createManualObject(ss2.str());
    scene_node_->attachObject(mManualObject);

    mManualObject->begin(mMaterial->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
    {
        // First triangle
        {
            // Bottom left
            mManualObject->position(0.0f, 0.0f, 0.0f);
            mManualObject->textureCoord(0.0f, 0.0f);
            mManualObject->normal(0.0f, 0.0f, 1.0f);

            // Top right
            mManualObject->position(resolution * width, resolution * height, 0.0f);
            mManualObject->textureCoord(1.0f, 1.0f);
            mManualObject->normal(0.0f, 0.0f, 1.0f);

            // Top left
            mManualObject->position(0.0f, resolution * height, 0.0f);
            mManualObject->textureCoord(0.0f, 1.0f);
            mManualObject->normal(0.0f, 0.0f, 1.0f);
        }

        // Second triangle
        {
            // Bottom left
            mManualObject->position(0.0f, 0.0f, 0.0f);
            mManualObject->textureCoord(0.0f, 0.0f);
            mManualObject->normal(0.0f, 0.0f, 1.0f);

            // Bottom right
            mManualObject->position(resolution * width, 0.0f, 0.0f);
            mManualObject->textureCoord(1.0f, 0.0f);
            mManualObject->normal(0.0f, 0.0f, 1.0f);

            // Top right
            mManualObject->position(resolution * width, resolution * height, 0.0f);
            mManualObject->textureCoord(1.0f, 1.0f);
            mManualObject->normal(0.0f, 0.0f, 1.0f);
        }
    }
    mManualObject->end();

    if (mDrawUnderProperty->getValue().toBool())
    {
        mManualObject->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
    }

    mRresolutionProperty->setValue(resolution);
    mWidthProperty->setValue(width);
    mHeightProperty->setValue(height);
    mPositionProperty->setVector(position);
    mOrientationProperty->setQuaternion(orientation);

    transformAerialMap();

    mLoaded = true;

    context_->queueRender();
}

void AerialMapDisplay::incomingAerialMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg)
{
    mUpdatedMap = msg;
    std::unique_lock<std::mutex> lock(mMutex);
    mNewMap = true;
}


void AerialMapDisplay::transformAerialMap()
{
    if (!mCurrentMap)
    {
        return;
    }

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!context_->getFrameManager()->transform(
            mFrame, rclcpp::Time(), mCurrentMap->info.origin, position, orientation))
    {
        // ROS_DEBUG("Error transforming map '%s' from frame '%s' to frame '%s'",
        //           qPrintable(getName()),
        //           mFrame.c_str(),
        //           qPrintable(fixed_frame_));

        setStatus(rviz_common::properties::StatusProperty::Error,
                  "Transform",
                  "No transform from [" + QString::fromStdString(mFrame) + "] to [" + fixed_frame_ + "]");
    }
    else
    {
        setStatus(rviz_common::properties::StatusProperty::Ok, "Transform", "Transform OK");
    }

    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);
}

void AerialMapDisplay::fixedFrameChanged()
{
    transformAerialMap();
}

void AerialMapDisplay::reset()
{
    Display::reset();

    clear();
    // Force resubscription so that the map will be re-sent
    updateTopic();
}
}   // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(AerialMapDisplay, rviz_common::Display)
