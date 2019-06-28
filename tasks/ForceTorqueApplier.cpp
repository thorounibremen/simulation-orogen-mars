/* Generated from orogen/lib/orogen/templates/tasks/IMU.cpp */

#include "ForceTorqueApplier.hpp"
#include "Plugin.hpp"
#include <mars/interfaces/sim/EntityManagerInterface.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <mars/interfaces/graphics/draw_structs.h>
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
    draw_item item;
    std::vector<long unsigned int> node_id;
    draw.drawItems.clear();
    auto n_it=wrenches.names.begin();
    auto w_it=wrenches.elements.begin();
    for (; n_it!=wrenches.names.end() && w_it!=wrenches.elements.end(); n_it++, w_it++) {
      node_id.clear();
      node_id.push_back(robot_entity->getNode(*n_it));
      if (node_id[0] == 0) {
        fprintf(stderr, "Node with name %s not found!\n", n_it->c_str());
      } else {
        control->nodes->applyForce(node_id[0], w_it->force);
        control->nodes->applyTorque(node_id[0], w_it->torque);
        if (visualize_wrenches && w_it->force.norm() > 0.01) {
          item.id = 0;
          item.type = DRAW_ARROW;
          item.point_size = 10;
          item.t_height = 0;
          item.texture = "";
          item.get_light = 0;
          item.start = control->nodes->getCenterOfMass(node_id);
          //force
          item.myColor.r = 1;
          item.myColor.g = 0;
          item.myColor.b = 0;
          item.myColor.a = 1;
          char label[100];
          sprintf(label, "(%f, %f, %f)",
            w_it->force.x(), w_it->force.y(),
            w_it->force.z()
          );
          item.label = std::string(label);
          item.end = item.start + w_it->force*0.01;
          draw.drawItems.push_back(item);
          //torque
          item.myColor.r = 0;
          item.myColor.g = 0;
          item.myColor.b = 1;
          item.myColor.a = 1;
          sprintf(label, "(%f, %f, %f)",
            w_it->torque.x(), w_it->torque.y(),
            w_it->torque.z()
          );
          item.label = std::string(label);
          item.end = item.start + w_it->torque*0.1;
          draw.drawItems.push_back(item);
        }
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

    draw.ptr_draw = (DrawInterface*) this;

    return true;
  }

  bool ForceTorqueApplier::startHook()
  {
    return ForceTorqueApplierBase::startHook();
  }

  void ForceTorqueApplier::updateHook()
  {
    fprintf(stderr, "test");
    ForceTorqueApplierBase::updateHook();

    _wrenches.readNewest(wrenches);

    //write outputs
    _input_wrenches.write(wrenches);
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
