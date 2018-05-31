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
    LOG_DEBUG_S << "RobotTeleportation " << "got robot";
    //get config properties
    pos_mode = _position_mode.get();
    LOG_DEBUG_S << "RobotTeleportation " << "got config";
    //save initial position
    target.id = 0;
    target.cfg = robot_entity->getConfig();
    //initiate target vector
    targets.push_back(target);
    LOG_DEBUG_S << "RobotTeleportation " << "got current state";
    //get target positions from file
    if (pos_mode == 1) {
      //load target config list
      ConfigVector::iterator mIt = target.cfg["teleportation_targets"].begin();
      unsigned int id = 1; //first item is initial position
      for (; mIt != target.cfg["teleportation_targets"].end(); ++mIt) {
        Target t;
        t.id = ++id;
        t.cfg = *mIt;
        targets.push_back(t);
      }
      LOG_DEBUG_S << "RobotTeleportation " << "read targets";
    }

    LOG_DEBUG_S << "RobotTeleportation "<< "Task configured! " << (pos_mode ? "(Manual)":"(Preconfigured)");

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

    //if triggered load new position
    if (_position_id.readNewest(curr_id) == RTT::NewData ||
        _position.readNewest(pos) == RTT::NewData ||
        _rotation.readNewest(rot) == RTT::NewData ||
        _anchor.readNewest(anchor) == RTT::NewData
       )
    {
      LOG_DEBUG_S << "RobotTeleportation"<< "updateHook triggered!";
      if (pos_mode == 1){
        if (curr_id >=targets.size()) {
          LOG_DEBUG_S << "RobotTeleportation"<< "given to high id " << curr_id << ">=" <<targets.size()<<"!";
          curr_id = 0;
        }
        target = targets[curr_id];
      } else {
        target.id = -1;
        target.cfg["position"][0] = pos[0];
        target.cfg["position"][1] = pos[1];
        target.cfg["position"][2] = pos[2];
        sRotation rot_e = quaternionTosRotation(rot);
        target.cfg["rotation"][0] = rot_e.alpha;
        target.cfg["rotation"][1] = rot_e.beta;
        target.cfg["rotation"][2] = rot_e.gamma;
        if (anchor) {
          target.cfg["anchor"] = "world";
        } else {
          target.cfg["anchor"] = "none";
        }
        fprintf(stderr, "anchor %s\n", target.cfg["anchor"].c_str());
      }
      robot_entity->setInitialPose(false, &target.cfg);

      LOG_DEBUG_S << "RobotTeleportation"<< "Robot teleportation done!";
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

