/* Generated from orogen/lib/orogen/templates/tasks/IMU.cpp */

#include "EntityFakeDetection.hpp"
#include "Plugin.hpp"
#include <mars/interfaces/sim/EntityManagerInterface.h>
#include <mars/interfaces/sim/SimulatorInterface.h>
#include <mars/utils/mathUtils.h>

using namespace mars;

using namespace mars;


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
  if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning

  /* todo
  //get transformation from inputs
  //get camera position
  */

  // get data from entities
  all_entities = control->entities->getEntities();
  //set general header
  Detection3DArray* detectionArray = new Detection3DArray(max_objects)
  detectionArray->header.stamp = base::Time::fromMilliseconds(control->sim->getTime());
  detectionArray->header.seq = seq++;
  //generate detections
  int i = 0;
  for (std::map<unsigned long, SimEntity*>::iterator iter = entities.begin();
       iter != entities.end() && i<max_objects; ++iter) {
    //Header
    detectionArray->detections[i].header.stamp = base::Time::fromMilliseconds(control->sim->getTime());
    detectionArray->detections[i].header.seq = seq++;
    //ObjectHypothesisWithPose
    rootId = iter->second->getRootestId("collision"); //returns the highest node with collision in the name
    detectionArray->detections[i].results[0].id = iter->first;
    detectionArray->detections[i].results[0].pose.pose.position = control->nodes->getPosition(rootId);
    detectionArray->detections[i].results[0].pose.pose.orientation = control->nodes->getRotation(rootId);
    //BoundingBox3D
    detectionArray->detections[i].bbox.center = detectionArray->detections[i].results[0].pose.pose.position;
    detectionArray->detections[i].bbox.size = control->nodes->getFullNode(rootId).ext;
    //PointCloud
    detectionArray->detections[i].source_cloud.header.stamp = base::Time::fromMilliseconds(control->sim->getTime());
    detectionArray->detections[i].source_cloud.header.seq = seq++;
    detectionArray->detections[i].source_cloud.width = control->nodes->getFullNode(rootId).mesh.vertexcount;
    detectionArray->detections[i].source_cloud.vertices = control->nodes->getFullNode(rootId).mesh.vertices;// REVIEW
    i++;
  }

  //write to rock outputs
  _detectionArray.write(*detectionArray);

}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See EntityFakeDetection.hpp for more detailed
// documentation about them.

// bool EntityFakeDetection::configureHook()
// {
//     if (! EntityFakeDetectionBase::configureHook())
//         return false;
//     return true;
// }
bool EntityFakeDetection::startHook()
{
    if (! EntityFakeDetectionBase::startHook())
        return false;
    return true;

    max_objects = _max_objects.get()
}
void EntityFakeDetection::updateHook()
{
    EntityFakeDetectionBase::updateHook();
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

