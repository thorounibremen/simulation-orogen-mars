/* Generated from orogen/lib/orogen/templates/tasks/IMU.cpp */

#include "ForceTorqueApplier.hpp"
#include "Plugin.hpp"
#include <mars/interfaces/sim/EntityManagerInterface.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/sim/SimEntity.h>
#include <mars/utils/misc.h>
#include <unistd.h>

#include <base-logging/Logging.hpp>


namespace mars {

  using namespace interfaces;
  using namespace utils;


  ForceTorqueApplier::ForceTorqueApplier(std::string const& name)
      : ForceTorqueApplierBase(name)
  {
  }

  ForceTorqueApplier::ForceTorqueApplier(std::string const& name, RTT::ExecutionEngine* engine)
      : ForceTorqueApplierBase(name, engine)
  {
  }

  ForceTorqueApplier::~ForceTorqueApplier()
  {
  }

  void ForceTorqueApplier::init()
  {
  }

  void ForceTorqueApplier::update(double delta_t)
  {
    for (unsigned int i = 0; i<node_ids.size(); i++) {
      if (apply[i]) {
        control->nodes->applyForce(node_ids[i], wrenches[i].force);
        control->nodes->applyTorque(node_ids[i], wrenches[i].torque);
      } else {
        control->nodes->applyForce(node_ids[i], Vector(0, 0, 0));
        control->nodes->applyTorque(node_ids[i], Vector(0, 0, 0));
      }
      if (visualize_wrenches) {
        // TODO
      }
    }
  }

  /// The following lines are template definitions for the various state machine
  // hooks defined by Orocos::RTT. See ForceTorqueApplier.hpp for more detailed
  // documentation about them.

  bool ForceTorqueApplier::configureHook()
  {
    if (! ForceTorqueApplierBase::configureHook()) return false;
    robot_name = _entity.get();
    visualize_wrenches = _visualize_wrenches.get();

    robot_entity = control->entities->getEntity(robot_name);

    return true;
  }

  bool ForceTorqueApplier::startHook()
  {
    return ForceTorqueApplierBase::startHook();
  }

  void ForceTorqueApplier::updateHook()
  {
    ForceTorqueApplierBase::updateHook();

    if (_link_names.readNewest(link_names) == RTT::NewData) {
      node_ids.clear();
      for (auto it=link_names.begin(); it!=link_names.end(); it++) {
        node_ids.push_back(robot_entity->getNode(*it));
      }
    }

    _wrenches.readNewest(wrenches);
    _apply.readNewest(apply);

  }

  // void ForceTorqueApplier::errorHook()
  // {
  //     ForceTorqueApplierBase::errorHook();
  // }

  void ForceTorqueApplier::stopHook()
  {
    ForceTorqueApplierBase::stopHook();
  }

  // void ForceTorqueApplier::cleanupHook()
  // {
  //     ForceTorqueApplierBase::cleanupHook();
  // }
}
