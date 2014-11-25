/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "DepthCamera.hpp"

using namespace mars;

DepthCamera::DepthCamera(std::string const& name)
    : DepthCameraBase(name)
{
}

DepthCamera::DepthCamera(std::string const& name, RTT::ExecutionEngine* engine)
    : DepthCameraBase(name, engine)
{
}

DepthCamera::~DepthCamera()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See DepthCamera.hpp for more detailed
// documentation about them.
bool DepthCamera::configureHook()
{
    
    if (! DepthCameraBase::configureHook())
        return false;
    return true;
}

bool DepthCamera::startHook()
{
        std::cout << "DepthCamera startHook" <<  std::endl;
    if (! DepthCameraBase::startHook())
        return false;
    image = new base::samples::DistanceImage(width, height);
    image->setSize(width, height);
    ro_ptr.reset(image);
    return true;
}

void DepthCamera::updateHook()
{
    DepthCameraBase::updateHook();
}

void DepthCamera::errorHook()
{
    DepthCameraBase::errorHook();
}

void DepthCamera::stopHook()
{
    DepthCameraBase::stopHook();
}

void DepthCamera::cleanupHook()
{
    DepthCameraBase::cleanupHook();
}

void DepthCamera::getData()
{	
    image = ro_ptr.write_access();
    camera->getDepthImage(image->data);
    
    // get the camera info for the intrinsic parameters of the virtual camera
    mars::interfaces::cameraStruct camInfo;
    camera->getCameraInfo(&camInfo);
    image->setIntrinsic(camInfo.scale_x, camInfo.scale_y, 
                        camInfo.center_x, camInfo.center_y );

    //TODO camera might be rotated

    //set attributes
    //image->time = base::Time::fromSeconds(lastUpdateTime);
    image->time = getTime();
    
    ro_ptr.reset(image);
    
    _distance_image.write(ro_ptr);
}
