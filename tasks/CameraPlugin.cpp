/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CameraPlugin.hpp"
#include <mars/sim/CameraSensor.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>

#include <base/Logging.hpp>

using namespace mars;

CameraPlugin::CameraPlugin(std::string const& name)
    : CameraPluginBase(name),
        sensor_id(0.0),
        width(0),
        height(0),
        lastUpdateTime(0),
        lastGrabTime(0),
        camera(NULL),
        frameDelay(0),
        isPeriodic(false)
{
}

CameraPlugin::CameraPlugin(std::string const& name, RTT::ExecutionEngine* engine)
    : CameraPluginBase(name, engine),
        sensor_id(0.0),
        width(0),
        height(0),
        lastUpdateTime(0),
        lastGrabTime(0),
        camera(NULL),
        frameDelay(0),
        isPeriodic(false)
{
}

CameraPlugin::~CameraPlugin()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See CameraPlugin.hpp for more detailed
// documentation about them.
bool CameraPlugin::configureHook()
{
    if (! mars::Plugin::configureHook())
        return false;


    return true;
}

bool CameraPlugin::startHook()
{


    if (! mars::Plugin::startHook())
        return false;

    sensor_id = control->sensors->getSensorID( _name.value() );
    if( !sensor_id ){
	    LOG_ERROR_S << "CameraPlugin" << "CameraPlugin: There is no camera by the name of " + _name.value() + " in the scene";
        return false;
    }
    LOG_DEBUG_S << "CameraPlugin" << "Camera '"<< _name.value() <<"' found!";

    camera = dynamic_cast<mars::sim::CameraSensor *>(control->sensors->getSimSensor(sensor_id));
    if( !camera){
	    LOG_ERROR_S << "CameraPlugin" << "CameraPlugin: Given sensor name is not a camera";
      return false;
    }

    camera->activateRendering();
    width = camera->getConfig().width;
    height = camera->getConfig().height;

    control->graphics->addGraphicsUpdateInterface(this);

    // Used in postGraphicsUpdate() to trigger the updateHook every 'frameDelay' ms.
    frameDelay = 1000 / camera->getConfig().updateRate;

    RTT::base::ActivityInterface* activity = this->getActivity();
    isPeriodic = activity->isPeriodic();
    return true;
}

void CameraPlugin::updateHook()
{
    mars::Plugin::updateHook();

    getData();
}

void CameraPlugin::errorHook()
{
    mars::Plugin::errorHook();
}



void CameraPlugin::stopHook()
{
    mars::Plugin::stopHook();
    camera->deactivateRendering();
}



void CameraPlugin::cleanupHook()
{
    mars::Plugin::cleanupHook();
}

void CameraPlugin::update(mars::interfaces::sReal time_ms)
{
    lastUpdateTime += time_ms;
}

void CameraPlugin::postGraphicsUpdate(void )
{
    // Frame rate is defined by the update rate of the module.
    if(isPeriodic)
        return;

    if((lastUpdateTime - lastGrabTime) < frameDelay)
	    return;

    // Triggers the updateHook/image request every 'frameDelay' ms.
	trigger();

    lastGrabTime = lastUpdateTime;
}

void CameraPlugin::getData(){
    //This method has to be implemented by the sublasses and is not allowed to be called
    assert(false);
}
