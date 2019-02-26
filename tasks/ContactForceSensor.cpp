/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ContactForceSensor.hpp"
#include <mars/sim/NodeContactForceSensor.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>

using namespace mars;

ContactForceSensor::ContactForceSensor(std::string const& name)
    : ContactForceSensorBase(name)
{
}

ContactForceSensor::ContactForceSensor(std::string const& name, RTT::ExecutionEngine* engine)
    : ContactForceSensorBase(name, engine)
{
}

ContactForceSensor::~ContactForceSensor()
{
}

bool ContactForceSensor::configureHook()
{
    if (! ContactForceSensorBase::configureHook())
        return false;

    wrench_mappings = _wrench_mappings.get();

    mars_ids.clear();
    sensor_names.clear();
    for(WrenchMapping w : wrench_mappings){
        if(std::find(sensor_names.begin(), sensor_names.end(), w.sensor_name) == sensor_names.end())
            sensor_names.push_back(w.sensor_name);
    }

    mars_ids.resize(sensor_names.size());
    wrenches.resize(wrench_mappings.size());
    for(size_t i = 0; i < wrench_mappings.size(); i++){
        wrenches.names[i] = wrench_mappings[i].sensor_name;
        wrenches[i].force.setConstant(std::numeric_limits<double>::quiet_NaN());
        wrenches[i].torque.setConstant(std::numeric_limits<double>::quiet_NaN());
    }

    return true;
}

bool ContactForceSensor::startHook()
{
    if (! ContactForceSensorBase::startHook())
        return false;
    return true;
}

void ContactForceSensor::updateHook()
{
    ContactForceSensorBase::updateHook();
}

void ContactForceSensor::errorHook()
{
    ContactForceSensorBase::errorHook();
}

void ContactForceSensor::stopHook()
{
    ContactForceSensorBase::stopHook();
}

void ContactForceSensor::cleanupHook()
{
    ContactForceSensorBase::cleanupHook();
}

void ContactForceSensor::init()
{
    for( size_t i=0; i < sensor_names.size(); ++i )
    {
        int marsId = control->sensors->getSensorID( sensor_names[i] );
        if( marsId )
            mars_ids[i] = marsId;
        else
            throw std::runtime_error("there is no sensor by the name of " +  sensor_names[i]);
    }
}

void ContactForceSensor::update(double delta_t)
{
    if(!isRunning()) return;


    for( size_t i = 0; i < mars_ids.size(); ++i )
    {
        mars::interfaces::BaseSensor* base = control->sensors->getSimSensor(mars_ids[i]);
        if(base)
        {
            mars::sim::NodeContactForceSensor* sensor = dynamic_cast<mars::sim::NodeContactForceSensor*>(base);
            if(sensor)
            {
                interfaces::sReal *sens_val = 0;
                sensor->getSensorData(&sens_val);

                for(const WrenchMapping &w : wrench_mappings){
                    size_t idx = wrenches.mapNameToIndex(w.sensor_name);
                    MapFTValue(w.ft_name, sens_val[w.sensor_index], wrenches[idx]);
                }
                wrenches.time = getTime();
            }
        }
        else
            throw std::runtime_error("There is no sensor with id " + std::to_string(mars_ids[i]));
    }

    _wrenches.write(wrenches);
}

void ContactForceSensor::MapFTValue(const std::string ft_name, const double value, base::Wrench& wrench){
    if(ft_name == "Fx")
        wrench.force(0) = value;
    else if(ft_name == "Fy")
        wrench.force(1) = value;
    else if(ft_name == "Fz")
        wrench.force(2) = value;
    else if(ft_name == "Tx")
        wrench.torque(0) = value;
    else if(ft_name == "Ty")
        wrench.torque(1) = value;
    else if(ft_name == "Tz")
        wrench.torque(2) = value;
    else
        throw std::runtime_error("Invalid Ft name: " + ft_name + ". Must be one of Fx, Fy, Fz, Tx, Ty");
}
