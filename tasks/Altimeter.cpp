/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Altimeter.hpp"
#include <mars/sim/RaySensor.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>

using namespace mars;

Altimeter::Altimeter(std::string const& name)
    : AltimeterBase(name)
{
}

Altimeter::Altimeter(std::string const& name, RTT::ExecutionEngine* engine)
    : AltimeterBase(name, engine)
{
}

Altimeter::~Altimeter()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Altimeter.hpp for more detailed
// documentation about them.

bool Altimeter::configureHook()
{
    if (! AltimeterBase::configureHook())
        return false;
    
    node_id = control->sensors->getSensorID(_node_name.get());
    sonar = dynamic_cast<mars::sim::RaySensor*>(control->sensors->getSimSensor(node_id));
    if(sonar==0){
        fprintf(stderr,"[FATAL] Given node name is not a RaySensor (Name: %s, internal id: %lu).",_node_name.get().c_str(), node_id);
        return false;
    }

    return true;
}
bool Altimeter::startHook()
{
    if (! AltimeterBase::startHook())
        return false;
    return true;
}
void Altimeter::updateHook()
{
    AltimeterBase::updateHook();

    std::vector<double> sensor_data = sonar->getSensorData();
    if(sensor_data.size() != 1){
        fprintf(stderr,"Warning Sensor data size should be one for GroundDistance but is: %lu\n",sensor_data.size());
    }

    gdist.position = Eigen::Vector3d(0.0,0.0,sensor_data[0]);
    _ground_distance.write(gdist);
}

void Altimeter::errorHook()
{
    AltimeterBase::errorHook();
}
void Altimeter::stopHook()
{
    AltimeterBase::stopHook();
}
void Altimeter::cleanupHook()
{
    AltimeterBase::cleanupHook();
}
