/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MarsTrigger.hpp"

using namespace mars;

MarsTrigger::MarsTrigger(std::string const& name)
    : MarsTriggerBase(name)
{
    pthread_mutex_init(&mutex, NULL);
    pthread_cond_init(&mutex_cv, NULL);
}

MarsTrigger::MarsTrigger(std::string const& name, RTT::ExecutionEngine* engine)
    : MarsTriggerBase(name, engine)
{
    pthread_mutex_init(&mutex, NULL);
    pthread_cond_init(&mutex_cv, NULL);
}

MarsTrigger::~MarsTrigger()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MarsTrigger.hpp for more detailed
// documentation about them.
bool MarsTrigger::configureHook()
{
    if (! mars::Plugin::configureHook())
        return false;
    return true;
}


bool MarsTrigger::startHook()
{
    if (! mars::Plugin::startHook())
        return false;
    return control->dataBroker->registerTriggeredReceiver(this,"mars_sim", "simTime","mars_sim/postPhysicsUpdate",1);
    
}


void MarsTrigger::updateHook()
{
    mars::Plugin::updateHook();
    if(!_do_step.get()) return; 
    RTT::TaskContext::updateHook();
    pthread_mutex_lock(&mutex);
    sim->singleStep();
    pthread_cond_wait(&mutex_cv, &mutex);
    pthread_mutex_unlock(&mutex);
}


void MarsTrigger::errorHook()
{
    mars::Plugin::errorHook();
}

void MarsTrigger::stopHook()
{
    mars::Plugin::stopHook();
}

void MarsTrigger::cleanupHook()
{
    mars::Plugin::cleanupHook();
}


void MarsTrigger::receiveData(
        const mars::data_broker::DataInfo& info,
        const mars::data_broker::DataPackage& package,
        int id) 
{
    pthread_mutex_lock(&mutex);
    pthread_cond_signal(&mutex_cv);
    pthread_mutex_unlock(&mutex);
}

