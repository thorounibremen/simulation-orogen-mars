/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RttMultiLevelLaserRangeFinder.hpp"
#include "MarsPlugin.hpp"
#include <mars/sim/MultiLevelLaserRangeFinder.h>
#include <base/time.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>

using namespace simulation;

RttMultiLevelLaserRangeFinder::RttMultiLevelLaserRangeFinder(std::string const& name)
    : RttMultiLevelLaserRangeFinderBase(name)
{
}

RttMultiLevelLaserRangeFinder::RttMultiLevelLaserRangeFinder(std::string const& name, RTT::ExecutionEngine* engine)
    : RttMultiLevelLaserRangeFinderBase(name, engine)
{
}

RttMultiLevelLaserRangeFinder::~RttMultiLevelLaserRangeFinder()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See RttMultiLevelLaserRangeFinder.hpp for more detailed
// documentation about them.

bool RttMultiLevelLaserRangeFinder::configureHook()
{
    if (! RttMultiLevelLaserRangeFinderBase::configureHook())
        return false;
    return true;
}
bool RttMultiLevelLaserRangeFinder::startHook()
{
    if (! RttMultiLevelLaserRangeFinderBase::startHook())
        return false;
        
    int sensor_id = control->sensors->getSensorID( _name.value() );
    if( !sensor_id ){
        std::cout <<  "There is no sensor by the name of " << _name.value() << " in the scene" << std::endl;
        return false;
    }

    sensor = dynamic_cast<mars::sim::MultiLevelLaserRangeFinder *>( control->sensors->getSimSensor( sensor_id ) );
    if( !sensor ){
        std::cerr  << "The sensor with " <<  _name.value() <<  " is not of the correct type (MultiLevelLaserRangeFinder)" << std::endl;
        return false;
    }

    //init the scan
    const mars::sim::MultiLevelLaserRangeFinderConfig &config(sensor->getConfig());
    
    scan.horizontal_scans.resize(config.numRaysHorizontal);
    
    scan.min_range = _min_range.get() * 1000;
    scan.max_range = _max_range.get() * 1000;
    
    assert(config.numRaysHorizontal > 0);
    assert(config.numRaysVertical > 0);
    
    double horAngleStep = 0;
    double verAngleStep = 0;
    if(config.numRaysHorizontal != 1)
        horAngleStep = config.horizontalOpeningAngle / (config.numRaysHorizontal -1);
    if(config.numRaysVertical != 1)
        verAngleStep = config.verticalOpeningAngle / (config.numRaysVertical - 1);
    
    double verStartAngle = config.verticalStartAngle;
    
    for(int h = 0; h < config.numRaysHorizontal; h++)
    {
        velodyne_lidar::MultilevelLaserScan::VerticalMultilevelScan &verScan(scan.horizontal_scans[h]);
        verScan.vertical_scans.resize(config.numRaysVertical);
        verScan.horizontal_angle = base::Angle::fromRad(h * horAngleStep);
        verScan.vertical_start_angle = base::Angle::fromRad(verStartAngle);
        verScan.vertical_angular_resolution = verAngleStep;
        
//         std::cout << "Hor Angle " << verScan.horizontal_angle << std::endl;
//         std::cout << "Vertical Start Angle " << verScan.vertical_start_angle << std::endl;
//         std::cout << "Vertical Angle step " << verScan.vertical_angular_resolution << " end is " << verScan.vertical_start_angle + base::Angle::fromRad(verScan.vertical_angular_resolution * (config.numRaysVertical - 1)) << std::endl;
    }
    
    
    return true;
}

void  RttMultiLevelLaserRangeFinder::update( double time )
{
    //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning
    if(!isRunning()) 
        return; 
    
    if(!sensor->gotNewData())
        return;
    
    scan.time = getTime();
    const std::vector<double> &ranges(sensor->getSensorData());
    
    if(ranges.empty())
        return;
    
    const mars::sim::MultiLevelLaserRangeFinderConfig &config(sensor->getConfig());
    
    for(int h = 0; h < config.numRaysHorizontal; h++)
    {
        velodyne_lidar::MultilevelLaserScan::VerticalMultilevelScan &verScan(scan.horizontal_scans[h]);
        verScan.time = scan.time;
        for(int v = 0; v < config.numRaysVertical; v++)
        {
            const double range = ranges[v + h * config.numRaysVertical] * 1000.0;
            if(range < scan.min_range)
            {
                verScan.vertical_scans[v].range = velodyne_lidar::MultilevelLaserScan::TOO_NEAR;
                continue;
            }

            if(range > scan.max_range)
            {
                verScan.vertical_scans[v].range = velodyne_lidar::MultilevelLaserScan::TOO_FAR;
                continue;
            }

            verScan.vertical_scans[v].range = range;
        }
    }
    
    _laser_scans.write(scan);
}

void RttMultiLevelLaserRangeFinder::updateHook()
{
    
    RttMultiLevelLaserRangeFinderBase::updateHook();
}
void RttMultiLevelLaserRangeFinder::errorHook()
{
    RttMultiLevelLaserRangeFinderBase::errorHook();
}
void RttMultiLevelLaserRangeFinder::stopHook()
{
    RttMultiLevelLaserRangeFinderBase::stopHook();
}
void RttMultiLevelLaserRangeFinder::cleanupHook()
{
    RttMultiLevelLaserRangeFinderBase::cleanupHook();
}
