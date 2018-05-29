/* Generated from orogen/lib/orogen/templates/tasks/IMU.cpp */

#include "RobotTeleportation.hpp"
#include "Plugin.hpp"
#include <mars/interfaces/sim/EntityManagerInterface.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/JointManagerInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/sim/SimEntity.h>

#include <base-logging/Logging.hpp>


namespace mars {

  using namespace interfaces;
  using namespace utils;


  RobotTeleportation::RobotTeleportation(std::string const& name)
      : RobotTeleportationBase(name)
  {
  }

  RobotTeleportation::RobotTeleportation(std::string const& name, RTT::ExecutionEngine* engine)
      : RobotTeleportationBase(name, engine)
  {
  }

  RobotTeleportation::~RobotTeleportation()
  {
  }

  void RobotTeleportation::init()
  {
  }

  void RobotTeleportation::update(double delta_t)
  {
  }

  /// The following lines are template definitions for the various state machine
  // hooks defined by Orocos::RTT. See RobotTeleportation.hpp for more detailed
  // documentation about them.

  bool RobotTeleportation::configureHook()
  {
    if (! RobotTeleportationBase::configureHook()) return false;
    //get robot data
    robot_name = _robot_name.get();
    robot_entity = control->entities->getEntity(robot_name);
    robot_root = robot_entity->getRootestId();
    //get config properties
    pos_mode = _position_mode.get();
    config = robot_entity->getConfig();
    //save initial position
    target.id = 0;
    target.pos = control->nodes->getPosition(robot_root);
    target.rot = control->nodes->getRotation(robot_root);
    //check if there is a joint between root and world
    joint_id = control->joints->getIDByNodeIDs(0, robot_root);
    if (joint_id != 0) {
      target.anchored = true;
      assert(joint_id != INVALID_ID);
      joint = control->joints->getFullJoint(joint_id);
    } else {
      target.anchored = false;
      joint = JointData("anchor_" + robot_name, JointType::JOINT_TYPE_FIXED, 0, robot_root);
    }
    targets.push_back(target);
    current_state = target;
    //get target positions from file
    if (pos_mode == 1) {
      //load target config list
      config = robot_entity->getConfig();
      ConfigVector::iterator mIt = config["teleportation_targets"].begin();
      unsigned int id = 1; //first item is initial position
      for (; mIt != config["teleportation_targets"].end(); ++mIt) {
        cmap = *mIt;
        Target t;

        t.id = id++;
        t.anchored = cmap.get("anchored",true);
        vectorFromConfigItem(&cmap["position"], &t.pos);
        quaternionFromConfigItem(&cmap["rotation"], &t.rot);

        targets.push_back(t);
      }
    }

    LOG_DEBUG_S << "RobotTeleportation"<< "Task configured! " << (pos_mode ? "(Manual)":"(Preconfigured)");

    return true;
  }

  bool RobotTeleportation::startHook()
  {
    return RobotTeleportationBase::startHook();
  }

  void RobotTeleportation::updateHook()
  {
    RobotTeleportationBase::updateHook();

    if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning
    LOG_DEBUG_S << "RobotTeleportation"<< "Running updateHook!";

    //if triggered load new position
    triggered = _trigger_teleport.readNewest(triggered);
    if (triggered) {
      if (pos_mode == 1){
        _position_id.readNewest(curr_id);
        target = targets[curr_id];
      } else {
        target.id = -1;
        _position.readNewest(target.pos);
        _rotation.readNewest(target.rot);
        _anchor_to_world.readNewest(target.anchored);
      }
      //remove the anchor if necessary
      if (current_state.anchored) {
        control->joints->removeJoint(joint_id);
      }
      //set root node to the new position and fix it there if configured
      control->nodes->setPosition(robot_root, target.pos);
      if (target.anchored) control->joints->addJoint(&joint);
      //write state to outputs
      _set_position.write(target.pos);
      _set_rotation.write(target.rot);
      _anchored.write(target.anchored);
      current_state = target;
    }
  }

  // void RobotTeleportation::errorHook()
  // {
  //     RobotTeleportationBase::errorHook();
  // }

  void RobotTeleportation::stopHook()
  {
    RobotTeleportationBase::stopHook();
  }

  // void RobotTeleportation::cleanupHook()
  // {
  //     RobotTeleportationBase::cleanupHook();
  // }
}

