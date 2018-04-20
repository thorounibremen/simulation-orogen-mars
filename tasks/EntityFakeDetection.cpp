/* Generated from orogen/lib/orogen/templates/tasks/IMU.cpp */

#include "EntityFakeDetection.hpp"
#include "Plugin.hpp"
#include <mars/interfaces/sim/EntityManagerInterface.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/sim/SimEntity.h>

namespace mars {

  using namespace interfaces;


  EntityFakeDetection::EntityFakeDetection(std::string const& name)
      : EntityFakeDetectionBase(name)
  {
  }

  EntityFakeDetection::EntityFakeDetection(std::string const& name, RTT::ExecutionEngine* engine)
      : EntityFakeDetectionBase(name, engine)
  {
  }

  EntityFakeDetection::~EntityFakeDetection()
  {
  }

  void EntityFakeDetection::init()
  {
  }

  void EntityFakeDetection::update(double delta_t)
  {
  }

  /// The following lines are template definitions for the various state machine
  // hooks defined by Orocos::RTT. See EntityFakeDetection.hpp for more detailed
  // documentation about them.

  bool EntityFakeDetection::configureHook()
  {
        if (! EntityFakeDetectionBase::configureHook())
      return false;
    return true;
  }

  bool EntityFakeDetection::startHook()
  {
      if (! EntityFakeDetectionBase::startHook())
          return false;

      // get data from entities
      all_entities = control->entities->subscribeToEntityCreation(nullptr);
      detectionArray = new Detection3DArray(all_entities->size());

      return true;
  }

  void EntityFakeDetection::updateHook()
  {
      EntityFakeDetectionBase::updateHook();

      if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning
      /* todo
      //get transformation from inputs
      //get camera position
      */
      //set general header
      detectionArray->header.stamp = base::Time::fromMilliseconds(control->sim->getTime());
      detectionArray->header.seq = seq++;
      //generate detections
      unsigned int i = 0;
      for (std::map<unsigned long, sim::SimEntity*>::const_iterator iter = all_entities->begin();
           iter != all_entities->end(); ++iter) {
        //Header
        detectionArray->detections[i].header.stamp = base::Time::fromMilliseconds(control->sim->getTime());
        detectionArray->detections[i].header.seq = seq++;
        //ObjectHypothesisWithPose
        unsigned int rootId = iter->second->getRootestId("collision"); //returns the highest node with collision in the name
        detectionArray->detections[i].results[0].id = (iter->first);
        detectionArray->detections[i].results[0].type = iter->second->getName();
        detectionArray->detections[i].results[0].pose.pose.position = control->nodes->getPosition(rootId);
        detectionArray->detections[i].results[0].pose.pose.orientation = control->nodes->getRotation(rootId);
        //BoundingBox3D
        iter->second->getBoundingBox(detectionArray->detections[i].bbox.center.position,
                                     detectionArray->detections[i].bbox.center.orientation,
                                     detectionArray->detections[i].bbox.size);
        //PointCloud
        detectionArray->detections[i].source_cloud.header.stamp = base::Time::fromMilliseconds(control->sim->getTime());
        detectionArray->detections[i].source_cloud.header.seq = seq++;
        detectionArray->detections[i].source_cloud.width = control->nodes->getFullNode(rootId).mesh.vertexcount;
        //detectionArray->detections[i].source_cloud.points = control->nodes->getFullNode(rootId).mesh.vertices;// REVIEW+
        i++;
      }
      //write to rock outputs
      _detectionArray.write(*detectionArray);
  }
  // void EntityFakeDetection::errorHook()
  // {
  //     EntityFakeDetectionBase::errorHook();
  // }
  void EntityFakeDetection::stopHook()
  {
      EntityFakeDetectionBase::stopHook();
  }
  // void EntityFakeDetection::cleanupHook()
  // {
  //     EntityFakeDetectionBase::cleanupHook();
  // }

}

