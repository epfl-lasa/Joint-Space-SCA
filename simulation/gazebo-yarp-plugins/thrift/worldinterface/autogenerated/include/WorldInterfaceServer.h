/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_WorldInterfaceServer
#define YARP_THRIFT_GENERATOR_WorldInterfaceServer

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <Color.h>
#include <Pose.h>

namespace GazeboYarpPlugins {
  class WorldInterfaceServer;
}


class GazeboYarpPlugins::WorldInterfaceServer : public yarp::os::Wire {
public:
  WorldInterfaceServer();
  /**
   * Make a sphere.
   * @param radius radius of the sphere [m]
   * @param pose pose of the sphere [m]
   * @param color color of the sphere
   * @param frame_name (optional) is specified, the pose will be relative to the specified fully scoped frame (e.g. MODEL_ID::FRAME_ID). Otherwise, world it will be used.
   * @param object_name (optional) assigns a name to the object.
   * @param gravity_enable (optional) enables gravity (default false)
   * @param collision_enable (optional) enables collision (default true)
   * @return returns a string that contains the name of the object in the world
   */
  virtual std::string makeSphere(const double radius, const Pose& pose, const Color& color, const std::string& frame_name = "", const std::string& object_name = "", const bool gravity_enable = 0, const bool collision_enable = 1);
  /**
   * Make a box.
   * @param width box width [m]
   * @param height box height[m]
   * @param thickness box thickness [m]
   * @param pose pose of the box [m]
   * @param color color of the box
   * @param frame_name (optional) is specified, the pose will be relative to the specified fully scoped frame (e.g. MODEL_ID::FRAME_ID). Otherwise, world it will be used.
   * @param object_name (optional) assigns a name to the object.
   * @param gravity_enable (optional) enables gravity (default false)
   * @param collision_enable (optional) enables collision (default true)
   * @return returns a string that contains the name of the object in the world
   */
  virtual std::string makeBox(const double width, const double height, const double thickness, const Pose& pose, const Color& color, const std::string& frame_name = "", const std::string& object_name = "", const bool gravity_enable = 0, const bool collision_enable = 1);
  /**
   * Make a cylinder.
   * @param radius radius of the cylinder [m]
   * @param length lenght of the cylinder [m]
   * @param pose pose of the cylinder [m]
   * @param color color of the cylinder
   * @param frame_name (optional) is specified, the pose will be relative to the specified fully scoped frame (e.g. MODEL_ID::FRAME_ID). Otherwise, world it will be used.
   * @param object_name (optional) assigns a name to the object.
   * @param gravity_enable (optional) enables gravity (default false)
   * @param collision_enable (optional) enables collision (default true)
   * @return returns a string that contains the name of the object in the world
   */
  virtual std::string makeCylinder(const double radius, const double length, const Pose& pose, const Color& color, const std::string& frame_name = "", const std::string& object_name = "", const bool gravity_enable = 0, const bool collision_enable = 1);
  /**
   * Make a reference frame.
   * @param size size of the frame [m]
   * @param pose pose of the frame [m]
   * @param color color of the frame
   * @param frame_name (optional) is specified, the pose will be relative to the specified fully scoped frame (e.g. MODEL_ID::FRAME_ID). Otherwise, world it will be used.
   * @param object_name (optional) assigns a name to the object.
   * @param gravity_enable (optional) enables gravity (default false)
   * @param collision_enable (optional) enables collision (default true)
   * @return returns a string that contains the name of the object in the world
   */
  virtual std::string makeFrame(const double size, const Pose& pose, const Color& color, const std::string& frame_name = "", const std::string& object_name = "", const bool gravity_enable = 0, const bool collision_enable = 1);
  /**
   * Change the color of an object
   * @param id object id
   * @param color color of the frame
   * @return returns true or false on success failure
   */
  virtual bool changeColor(const std::string& id, const Color& color);
  /**
   * Set new object pose.
   * @param id object id
   * @param pose new pose
   * @param frame_name (optional) is specified, the pose will be relative to the specified fully scoped frame (e.g. MODEL_ID::FRAME_ID). Otherwise, world it will be used.
   * @return returns true or false on success failure
   */
  virtual bool setPose(const std::string& id, const Pose& pose, const std::string& frame_name = "");
  /**
   * Enable/disables gravity for an object
   * @param id object id
   * @param enable 1 to enable gravity, 0 otherwise
   * @return returns true or false on success failure
   */
  virtual bool enableGravity(const std::string& id, const bool enable);
  /**
   * Enable/disables collision detection for an object
   * @param id object id
   * @param enable 1 to enable collision detection, 0 otherwise
   * @return returns true or false on success failure
   */
  virtual bool enableCollision(const std::string& id, const bool enable);
  /**
   * Get object pose.
   * @param id string that identifies object in gazebo (returned after creation)
   * @param frame_name (optional) is specified, the pose will be relative to the specified fully scoped frame (e.g. MODEL_ID::FRAME_ID). Otherwise, world it will be used.
   * @return returns value of the pose in the world reference frame
   */
  virtual Pose getPose(const std::string& id, const std::string& frame_name = "");
  /**
   * Load a model from file.
   * @param id string that specifies the name of the model
   * @return returns true/false on success failure.
   */
  virtual bool loadModelFromFile(const std::string& filename);
  /**
   * Delete an object.
   * @param id string that identifies object in gazebo (returned after creation)
   * @return returns true/false on success failure.
   */
  virtual bool deleteObject(const std::string& id);
  /**
   * Delete all objects in the world.
   */
  virtual bool deleteAll();
  /**
   * List id of all objects that have been added to the world.
   * @return return a list of string containing the id of the objects
   */
  virtual std::vector<std::string>  getList();
  /**
   * Attach an object to a link of the robot.
   * @param id string that identifies object in gazebo (returned after creation)
   * @param link_name name of a link of the robot
   * @return true if success, false otherwise
   */
  virtual bool attach(const std::string& id, const std::string& link_name);
  /**
   * Detach a previously attached object.
   * @param id string that identifies object in gazebo (returned after creation)
   * @return true if success, false otherwise
   */
  virtual bool detach(const std::string& id);
  /**
   * Change the names of an object.
   * @param old_name string that identifies object in gazebo
   * @param new_name string that will be used as new name
   * @return true if success, false otherwise
   */
  virtual bool rename(const std::string& old_name, const std::string& new_name);
  virtual bool read(yarp::os::ConnectionReader& connection) override;
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
