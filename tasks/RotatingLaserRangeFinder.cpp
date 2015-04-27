/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RotatingLaserRangeFinder.hpp"

#include <mars/sim/RotatingRaySensor.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>

using namespace mars;

RotatingLaserRangeFinder::RotatingLaserRangeFinder(std::string const& name)
    : RotatingLaserRangeFinderBase(name), mSensorID(0), mSensor(NULL)
{
}

RotatingLaserRangeFinder::RotatingLaserRangeFinder(std::string const& name, RTT::ExecutionEngine* engine)
    : RotatingLaserRangeFinderBase(name, engine), mSensorID(0), mSensor(NULL)
{
}

RotatingLaserRangeFinder::~RotatingLaserRangeFinder()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See RotatingLaserRangeFinder.hpp for more detailed
// documentation about them.

bool RotatingLaserRangeFinder::configureHook()
{
    if (! RotatingLaserRangeFinderBase::configureHook())
        return false;
    return true;
}
bool RotatingLaserRangeFinder::startHook()
{
    if (! RotatingLaserRangeFinderBase::startHook())
        return false;
           
    mSensorID = control->sensors->getSensorID( _name.value() );
    if( !mSensorID ){
        std::cout <<  "There is no sensor by the name of " << _name.value() << 
                " in the scene" << std::endl;
        return false;
    }

    mSensor = dynamic_cast<mars::sim::RotatingRaySensor*>(control->sensors->getSimSensor(mSensorID));
    if( !mSensor ){
        std::cerr  << "The sensor with " <<  _name.value() <<  
                " is not of the correct type (RotatingRaySensor)" << std::endl;
        return false;
    }
    
    return true;
}
void RotatingLaserRangeFinder::updateHook()
{
    RotatingLaserRangeFinderBase::updateHook();

    // Seems Plugin is set up but not active yet, we are not sure that we 
    // are initialized correctly so retuning
    if(!isRunning()) {
        return; 
    }

    base::samples::Pointcloud pointcloud;
    pointcloud.time = getTime();
    
    std::vector<mars::utils::Vector> data;
    if(mSensor->getPointcloud(data)) {
        // TODO Min/max is actually already part of the sensor
        std::vector<mars::utils::Vector>::iterator it = data.begin();
        for(; it != data.end(); it++) {
            int len_ray = it->norm();
            if(len_ray >= _min_range.get() && len_ray <= _max_range.get()) {
                base::Vector3d vec((*it)[0], (*it)[1], (*it)[2]);
                pointcloud.points.push_back(vec);
            }
        }
        _pointcloud.write(pointcloud);
    }
}

void RotatingLaserRangeFinder::errorHook()
{
    RotatingLaserRangeFinderBase::errorHook();
}
void RotatingLaserRangeFinder::stopHook()
{
    RotatingLaserRangeFinderBase::stopHook();
}
void RotatingLaserRangeFinder::cleanupHook()
{
    RotatingLaserRangeFinderBase::cleanupHook();
}

void RotatingLaserRangeFinder::update(double delta_t) {

}
