/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "EntityBoundingBox.hpp"
#include <mars/interfaces/sim/EntityManagerInterface.h>
#include <mars/interfaces/sim/SimulatorInterface.h>
#include <mars/utils/mathUtils.h>

using namespace mars;

using namespace mars;


EntityBoundingBox::EntityBoundingBox(std::string const& name)
    : EntityBoundingBoxBase(name)
{
}

EntityBoundingBox::EntityBoundingBox(std::string const& name, RTT::ExecutionEngine* engine)
    : EntityBoundingBoxBase(name, engine)
{
}

EntityBoundingBox::~EntityBoundingBox()
{
}

void EntityBoundingBox::init()
{

}

void EntityBoundingBox::update(double delta_t)
{
  if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning

  // get data from entities
  all_entities = control->entities->getEntities();

  detectionArray.header.stamp = base::Time::fromMilliseconds(control->sim->getTime());
  detectionArray.header.seq = seq++;

  int i = 0;
  for (std::map<unsigned long, SimEntity*>::iterator iter = entities.begin();
       iter != entities.end() && i<20; ++iter) {
    //Header
    detectionArray.detections[i].header.stamp = base::Time::fromMilliseconds(control->sim->getTime());
    detectionArray.detections[i].header.seq = seq++;
    //ObjectHypothesisWithPose
    rootId = iter->second->getRootestId("collision"); //returns the highest node with collision in the name
    detectionArray.detections[i].results[0].id = iter->first;
    detectionArray.detections[i].results[0].pose.pose.position = control->nodes->getPosition(rootId);
    detectionArray.detections[i].results[0].pose.pose.orientation = control->nodes->getRotation(rootId);
    //BoundingBox3D REVIEW
    detectionArray.detections[i].bbox.center = detectionArray.detections[i].results[0].pose.pose.position;
    detectionArray.detections[i].bbox.size = control->nodes->getFullNode(rootId).ext;
    /*//PointCloud TODO
    detectionArray.detections[i].source_cloud =
    */
    i++;
  }



}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See EntityBoundingBox.hpp for more detailed
// documentation about them.

// bool EntityBoundingBox::configureHook()
// {
//     if (! EntityBoundingBoxBase::configureHook())
//         return false;
//     return true;
// }
bool EntityBoundingBox::startHook()
{
    if (! EntityBoundingBoxBase::startHook())
        return false;
    return true;
}
void EntityBoundingBox::updateHook()
{
    EntityBoundingBoxBase::updateHook();
}
// void EntityBoundingBox::errorHook()
// {
//     EntityBoundingBoxBase::errorHook();
// }
void EntityBoundingBox::stopHook()
{
    EntityBoundingBoxBase::stopHook();
}
// void EntityBoundingBox::cleanupHook()
// {
//     EntityBoundingBoxBase::cleanupHook();
// }

}

