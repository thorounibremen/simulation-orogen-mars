/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Trigger.hpp"
#include <mars/interfaces/sim/ControlCenter.h>

using namespace mars;

Trigger::Trigger(std::string const& name)
    : TriggerBase(name)
{
    pthread_mutex_init(&mutex, NULL);
    pthread_cond_init(&mutex_cv, NULL);
}

Trigger::Trigger(std::string const& name, RTT::ExecutionEngine* engine)
    : TriggerBase(name, engine)
{
    pthread_mutex_init(&mutex, NULL);
    pthread_cond_init(&mutex_cv, NULL);
}

Trigger::~Trigger()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Trigger.hpp for more detailed
// documentation about them.
bool Trigger::configureHook()
{
    if (! mars::Plugin::configureHook())
        return false;
    return true;
}


bool Trigger::startHook()
{
    if (! mars::Plugin::startHook())
        return false;
    return control->dataBroker->registerTriggeredReceiver(this,"mars_sim", "simTime","mars_sim/postPhysicsUpdate",1);
    
}


void Trigger::updateHook()
{
    mars::Plugin::updateHook();
    if(!_do_step.get()) return; 
    RTT::TaskContext::updateHook();
    pthread_mutex_lock(&mutex);
    sim->singleStep();
    pthread_cond_wait(&mutex_cv, &mutex);
    pthread_mutex_unlock(&mutex);
}


void Trigger::errorHook()
{
    mars::Plugin::errorHook();
}

void Trigger::stopHook()
{
    mars::Plugin::stopHook();
}

void Trigger::cleanupHook()
{
    mars::Plugin::cleanupHook();
}


void Trigger::receiveData(
        const mars::data_broker::DataInfo& info,
        const mars::data_broker::DataPackage& package,
        int id) 
{
    pthread_mutex_lock(&mutex);
    pthread_cond_signal(&mutex_cv);
    pthread_mutex_unlock(&mutex);
}

