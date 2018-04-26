/* Generated from orogen/lib/orogen/templates/tasks/IMU.cpp */

#include "EntityFakeDetection.hpp"
#include "Plugin.hpp"
#include <mars/interfaces/sim/EntityManagerInterface.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/sim/SimEntity.h>

#include <base/Logging.hpp>

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

    frame_id = FrameId(_frame_id.get());
    visible_if = mars::sim::ViewMode(_visible_if.get());
    use_camera = (bool) _use_camera.get();

    if (use_camera == false) {
      frame_id = FrameId::GLOBAL;
    }

    LOG_DEBUG_S << "EntityFakeDetection"<< "Task configured! " << (use_camera ? "(Using Camera)":"(Not Using Camera)");

    return true;
  }

  bool EntityFakeDetection::startHook()
  {
    if (use_camera == true) {
      if (!EntityFakeDetectionBase::startHook()) {
        if (!camera) {
          LOG_ERROR_S << "EntityFakeDetection"<< "Start of CameraPlugin failed! Using without camera!";
          use_camera = false;
        }
      }
    }

    //get all entities
    visible_entities = *(control->entities->subscribeToEntityCreation(nullptr));
    detectionArray = new Detection3DArray(visible_entities.size());

    LOG_DEBUG_S << "EntityFakeDetection"<< "Task started!";

    return true;
  }

  void EntityFakeDetection::updateHook()
  {
      EntityFakeDetectionBase::updateHook();

      if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning
      LOG_DEBUG_S << "EntityFakeDetection"<< "Running updateHook!";

      if (use_camera == true) { // then fill the buffer only with the entities seen by the camera
        camera->getEntitiesInView(visible_entities, visible_if);
        LOG_DEBUG_S << "1";
        if (visible_entities.size()!=detectionArray->detections.size()) {
          LOG_DEBUG_S << "2";
          delete detectionArray;
          LOG_DEBUG_S << "3";
          detectionArray = new Detection3DArray(visible_entities.size());
          LOG_DEBUG_S << "4";
        }
      } else {
        frame_id = FrameId::GLOBAL;
        visible_if = mars::sim::ViewMode::NOTHING;
      }

      LOG_DEBUG_S<<"EntityFakeDetection"<< "got entities updateHook()";
      //set general header
      detectionArray->header.stamp = base::Time::fromMilliseconds(control->sim->getTime());
      detectionArray->header.seq = seq++;
      detectionArray->header.frame_id = frame_id;
      //generate detections
      utils::Vector center;
      utils::Quaternion rotation;
      unsigned int i = 0;
      for (std::map<unsigned long, sim::SimEntity*>::iterator iter = visible_entities.begin();
          iter != visible_entities.end(); ++iter) {
        //Header
        detectionArray->detections[i].header.stamp = base::Time::fromMilliseconds(control->sim->getTime());
        detectionArray->detections[i].header.seq = seq++;
        detectionArray->detections[i].header.frame_id = frame_id;
        //BoundingBox3D
        iter->second->getBoundingBox(center,
                                     rotation,
                                     detectionArray->detections[i].bbox.size);
        detectionArray->detections[i].bbox.center.position = center;
        detectionArray->detections[i].bbox.center.orientation = rotation;
        //ObjectHypothesisWithPose
        detectionArray->detections[i].results[0].id = iter->first;
        detectionArray->detections[i].results[0].type = iter->second->getName();
        detectionArray->detections[i].results[0].pose.pose.position = center;
        detectionArray->detections[i].results[0].pose.pose.orientation = rotation;
        //PointCloud
        detectionArray->detections[i].source_cloud.header.stamp = base::Time::fromMilliseconds(control->sim->getTime());
        detectionArray->detections[i].source_cloud.header.seq = seq++;
        detectionArray->detections[i].source_cloud.header.frame_id = frame_id;
        //TODO detectionArray->detections[i].source_cloud.width = control->nodes->getFullNode(rootId).mesh.vertexcount;
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

  void EntityFakeDetection::getData()
  {
  }
}

