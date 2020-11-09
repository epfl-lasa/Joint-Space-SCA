#ifndef MY_PLUGIN_H
#define MY_PLUGIN_H

#include <boost/bind.hpp>
#include <vector>
#include <gazebo-9/gazebo/gazebo.hh>
#include <gazebo-9/gazebo/common/common.hh>
#include <gazebo-9/gazebo/physics/Model.hh>
#include <gazebo-9/gazebo/common/Assert.hh>
#include <gazebo-9/gazebo/common/Console.hh>
#include <gazebo-9/gazebo/common/Time.hh>
#include <gazebo-9/gazebo/physics/ContactManager.hh>
#include <gazebo-9/gazebo/physics/Collision.hh>
#include <gazebo-9/gazebo/physics/Joint.hh>
#include <gazebo-9/gazebo/physics/Model.hh>
#include <gazebo-9/gazebo/physics/PhysicsEngine.hh>
#include <gazebo-9/gazebo/physics/SurfaceParams.hh>
#include <gazebo-9/gazebo/physics/World.hh>
#include <gazebo-9/gazebo/rendering/RenderingIface.hh>
#include <gazebo-9/gazebo/rendering/Scene.hh>
#include <gazebo-9/gazebo/rendering/UserCamera.hh>
#include <gazebo-9/gazebo/rendering/Visual.hh>
#include <gazebo-9/gazebo/util/LogRecord.hh>
#include <gazebo-9/gazebo/transport/Node.hh>
#include <gazebo-9/gazebo/transport/Publisher.hh>
#include <gazebo-9/gazebo/physics/Link.hh>

namespace gazebo
{
class changeColor : public WorldPlugin
{
public:
    virtual void Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/);
    std::vector<int> was_collided;
private:
    void on_msg(ConstGzString_VPtr &_msg);
    // Pointer to the model
    physics::ModelPtr model;
    physics::WorldPtr world;
    transport::PublisherPtr pub_visual;
    transport::SubscriberPtr sub_collisions;
    event::ConnectionPtr updateConnection;
    physics::Link_V links_list;
protected: 
    gazebo::transport::NodePtr gzNode;
    // Pointer to the update event connection
    //event::ConnectionPtr updateConnection;
};
}

#endif // MY_PLUGIN_H