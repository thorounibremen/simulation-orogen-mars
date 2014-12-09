#include "Task.hpp"
#include <mars/sim/Simulator.h>
#include <mars/utils/Thread.h>
#include <mars/utils/mathUtils.h>
#include <mars/interfaces/sim/SimulatorInterface.h>

#include <mars/tasks/MarsControl.hpp>
#include <mars/gui/MarsGui.h>
#include <mars/main_gui/MainGUI.h>
#include <mars/main_gui/GuiInterface.h>
#include <mars/cfg_manager/CFGManagerInterface.h>
#include <mars/graphics/GraphicsManager.h>
#include <mars/app/GraphicsTimer.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>

#include <mars/sim/SimMotor.h>
#include <mars/interfaces/sim/MotorManagerInterface.h>

//#include <mars/multisim-plugin/MultiSimPlugin.h>

#include <mars/lib_manager/LibManager.h>
#include <QApplication>
#include <QPlastiqueStyle>

#include <base/logging.h>

using namespace mars;
using namespace mars;

mars::interfaces::SimulatorInterface *Task::simulatorInterface = 0;
mars::Task *Task::taskInterface = 0;
mars::app::GraphicsTimer *Task::graphicsTimer = 0;
mars::lib_manager::LibManager* Task::libManager = 0; 

Task::Task(std::string const& name)
    : TaskBase(name)
    , multisimPlugin(0)
{
    Task::taskInterface = this;
    setlocale(LC_ALL,"C"); //Make sure english Encodings are used
    setenv("LANG","C",true);
    app = 0;
    _gravity.set(Eigen::Vector3d(0,0,-9.81));
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
    , multisimPlugin(0)
{
    Task::taskInterface = this;
    app = 0;
    setlocale(LC_ALL,"C"); //Make sure english Encodings are used
    setenv("LANG","C",true);
}

Task::~Task()
{
    delete app;
}


void Task::loadScene(::std::string const & path)
{
    if(simulatorInterface){
        simulatorInterface->loadScene(path, path,true,true);
    }else{
        RTT::log(RTT::Error) << "Simulator not yet started cout not load scenefile" << RTT::endlog();        
    }
}

mars::interfaces::SimulatorInterface* Task::getSimulatorInterface()
{
    return simulatorInterface;
}

mars::Task* Task::getTaskInterface(){
    return taskInterface;
}

void* Task::startTaskFunc(void* argument)
{
    setlocale(LC_ALL,"C"); //Make sure english Encodings are used
    setenv("LANG","C",true);
    TaskArguments* marsArguments = static_cast<TaskArguments*>(argument);

    Task* mars = marsArguments->mars;

    // Using the 'command-line' interface to pass
    // arguments to mars interface
    // set the option to "" if it does not require further args
    std::vector<Option> rawOptions = marsArguments->raw_options;
       
    if(marsArguments->controller_port > 0)
    {
        char buffer[10];
        sprintf(buffer, "%d", marsArguments->controller_port);
        Option controllerPortOption("-c", std::string(buffer));
        rawOptions.push_back(controllerPortOption);
    }

    char** argv = mars->setOptions(rawOptions);
    int count = mars->getOptionCount(rawOptions);
    // Plus one for executable name
    for(int i = 0; i < count + 1; i++)
    {
        RTT::log(RTT::Info) << "Simulator: argument #" << i << " " << argv[i] << RTT::endlog();
    }
 
    // Prepare Qt Application Thread which is required  
    // for core mars and gui
    int argc = count + 1;
    if(!Task::getTaskInterface()->app){
        //Initialize Qapplication only once! and keep the instance
        Task::getTaskInterface()->app = new QApplication(argc, argv);
        Task::getTaskInterface()->app->setStyle(new QPlastiqueStyle);
    }

    setlocale(LC_ALL,"C");
    setenv("LANG","C",true);
    struct lconv* locale = localeconv();
    RTT::log(RTT::Info) << "Active locale (LC_ALL): " << RTT::endlog();
      
    if( *(locale->decimal_point) != '.')
    {
        RTT::log(RTT::Error) << "Current locale conflicts with mars" << RTT::endlog();
        marsArguments->failed_to_init = true;
        return 0;
    }

    // Prepare the LibManager and required configuration files
    libManager = new mars::lib_manager::LibManager();

    std::string corelibsConfigPath;
    // If the graphical interface should be disabled, the configuration 
    // needs to exclude the shared library with graphics options
    // Thus, the nogui configuration file needs to be used for the startup 
    if(marsArguments->enable_gui)
    {
        corelibsConfigPath = marsArguments->config_dir + "/core_libs.txt";
    } else {
        corelibsConfigPath = marsArguments->config_dir + "/core_libs-nogui.txt";
    }

    libManager->loadConfigFile(corelibsConfigPath);

    // Setting the configuration directory and loading the preferences
    mars::lib_manager::LibInterface* lib = libManager->getLibrary(std::string("cfg_manager"));
    if(lib)
    {
 	cfg_manager::CFGManagerInterface* cfg = dynamic_cast<cfg_manager::CFGManagerInterface*>(lib);
        if(cfg)
        {

            cfg_manager::cfgPropertyStruct configPath;
            configPath = cfg->getOrCreateProperty("Config", "config_path", marsArguments->config_dir);

	    // overriding any defaults 
            configPath.sValue = marsArguments->config_dir;
            cfg->setProperty(configPath);
 	    /*
            if(!cfg->setProperty(configPath))
            {
                RTT::log(RTT::Error) << "Configuration path property could not be set" << RTT::endlog();
                exit(1);
            } 
            */
            configPath = cfg->getOrCreateProperty("Config", "config_path", marsArguments->config_dir);

            if(configPath.sValue != marsArguments->config_dir)
            {
		RTT::log(RTT::Error) << "CRITICAL (cause abort): Property was not set correctly: " << configPath.sValue << RTT::endlog();
                marsArguments->failed_to_init = true;
                return 0;
            } else {
                RTT::log(RTT::Error) << "Configuration path property set to " << configPath.sValue << RTT::endlog();
            }

            std::string loadFile = configPath.sValue;
            loadFile.append("/mars_Preferences.yaml");
            cfg->loadConfig(loadFile.c_str());
            loadFile = configPath.sValue;
            loadFile.append("/mars_Simulator.yaml");
            cfg->loadConfig(loadFile.c_str());

            loadFile = configPath.sValue;
            loadFile.append("/mars_Physics.yaml");
            cfg->loadConfig(loadFile.c_str());
            
            bool loadLastSave = false;
            cfg->getPropertyValue("Config", "loadLastSave", "value",
                                           &loadLastSave);
            if (loadLastSave) 
            {
                loadFile = configPath.sValue;
                loadFile.append("/mars_saveOnClose.yaml");
                cfg->loadConfig(loadFile.c_str());
            }      
        } else {
            RTT::log(RTT::Error) << "Error casting to cfg_dfki" << RTT::endlog();
        }
    } else {
        RTT::log(RTT::Error) << "Could not load library cfg_dfki" << RTT::endlog();
    }

    lib = libManager->getLibrary("mars_sim");
    if(!lib)
    {
        RTT::log(RTT::Error) << "CRITICAL (cause abort) Simulation library failed to load" << RTT::endlog();
        RTT::log(RTT::Error) << "Configuration loaded from " << corelibsConfigPath << RTT::endlog();
        marsArguments->failed_to_init = true;
        return 0;
    }


    // Prepare the mars instance, load the argument and run
    mars->simulatorInterface = dynamic_cast<sim::Simulator*>(lib); 
    if(!mars->simulatorInterface)
    {
        RTT::log(RTT::Error) << "CRITICAL (cause abort) Simulation could not be retrieved via lib_manager" << RTT::endlog();
        marsArguments->failed_to_init = true;
        return 0;
    }

    mars->simulatorInterface->readArguments(count + 1, argv);
    std::string cmd;
    for(int i = 0; i < count+1;++i)
    {
        cmd += std::string(argv[i]);
        cmd += " ";
    }

    RTT::log(RTT::Info) << "Starting mars with: " << cmd << RTT::endlog();


    // if we have a main gui, show it 
    if(marsArguments->enable_gui)
    {

        gui::MarsGui *marsGui = NULL;
        lib = libManager->getLibrary("mars_gui");
        if(lib)
        {
            if( (marsGui = dynamic_cast<gui::MarsGui*>(lib)) )
            {
                marsGui->setupGui();
            }
        } else {
            RTT::log(RTT::Error) << "CRITICAL (cause abort) Simulator: mars_gui not found, cannot show GUI" << RTT::endlog();
            marsArguments->failed_to_init = true;
            return 0;
        }

        main_gui::MainGUI* mainGui;
        lib = libManager->getLibrary("main_gui");
        if(lib && (mainGui = dynamic_cast<main_gui::MainGUI*>(lib)) )
        {
            // all good
        } else {
            RTT::log(RTT::Error) << "CRITICAL (cause abort) Simulator: gui_core not found, cannot show GUI" << RTT::endlog();
            marsArguments->failed_to_init = true;
            return 0;
        }


        lib = libManager->getLibrary("mars_graphics");
        if(lib) 
        {
            if( (Task::getTaskInterface()->marsGraphics = dynamic_cast<graphics::GraphicsManager*>(lib)) )
            {
                // init osg
                //initialize graphicsFactory
                Task::getTaskInterface()->marsGraphics->initializeOSG(NULL);
                void* widget = Task::getTaskInterface()->marsGraphics->getQTWidget(1);	
                if (widget && mainGui) 
                {
                    //control->gui->addDockWidget((void*)newWidget,1);
                    mainGui->mainWindow_p()->setCentralWidget((QWidget*)widget);
                    ((QWidget*)widget)->show();
                }     
            }
        }
        
        mars->simulatorInterface->runSimulation();

        mainGui->show();
        
    } else {
        mars->simulatorInterface->runSimulation();
    }

    // Loading libraries that are specified in other_libs.txt
    std::string addonsConfigPath = marsArguments->config_dir + "/other_libs.txt";
    libManager->loadConfigFile(addonsConfigPath);
    /*libManager->getAllLibraryNames(&lib_names);
    for(std::list<std::string>::iterator it = lib_names.begin(); it != lib_names.end(); ++it){
    }*/

    // set mars properties, if specified
    lib = libManager->getLibrary(std::string("cfg_manager"));
    if(lib)
    {
        cfg_manager::CFGManagerInterface* cfg = dynamic_cast<cfg_manager::CFGManagerInterface*>(lib);
        if(cfg){
            std::vector<SimulationProperty> props = marsArguments->mars_property_list;
            for(std::vector<SimulationProperty>::iterator prop_it = props.begin(); prop_it != props.end(); ++prop_it){
                // get or create property
                cfg_manager::cfgPropertyStruct cfg_prop_struct;
                cfg_prop_struct = cfg->getOrCreateProperty(prop_it->lib_name, prop_it->property_name, prop_it->value);

                // overriding any defaults
                cfg_prop_struct.sValue = prop_it->value;
                cfg->setProperty(cfg_prop_struct);
                printf("setting property %s\n", cfg_prop_struct.sValue.c_str());
            }
        }
    }

    // GraphicsTimer allows to update the graphics interface 
    // every 10 ms
    Task::graphicsTimer = new app::GraphicsTimer(Task::getTaskInterface()->marsGraphics, mars->simulatorInterface);
    Task::graphicsTimer->run();

    if(marsArguments->add_floor){
        mars->simulatorInterface->getControlCenter()->nodes->createPrimitiveNode("Boden",mars::interfaces::NODE_TYPE_PLANE,false,mars::utils::Vector(0,0,0.0),mars::utils::Vector(600,600,0));
    }
    int result = mars->simulatorInterface->getControlCenter()->dataBroker->registerTriggeredReceiver(mars,"mars_sim", "simTime","mars_sim/postPhysicsUpdate",1);
    (void)result;
    assert(result);
    
    // is realtime calc requested?
    if(marsArguments->realtime_calc){
    	mars->simulatorInterface->getControlCenter()->cfg->setPropertyValue("Simulator", "realtime calc", "value", marsArguments->realtime_calc);
    }
    
    // Synchronize with configureHook
    marsArguments->initialized = true;
    Task::getTaskInterface()->app->exec();
   
   
    libManager->releaseLibrary("mars_graphics");
    libManager->releaseLibrary("main_gui");
    libManager->releaseLibrary("mars_gui");
    libManager->releaseLibrary("mars_sim");
    libManager->releaseLibrary("cfg_manager");
   
    //Workaround release waht whereever is acquired
    libManager->releaseLibrary("data_broker");
    libManager->releaseLibrary("avalonplugin");
    libManager->releaseLibrary("mars_sim");
    libManager->releaseLibrary("main_gui");

    delete Task::graphicsTimer;
    delete libManager;
    //Do not delete the QApplication it does not like it to be restarted
    std::cout << "Qapplication exec ended" << std::endl;

    return 0;
}

int Task::getOptionCount(const std::vector<Option>& options)
{
    std::vector<Option>::const_iterator it;

    // First just counting the number of arguments
    int count = 0;
    for(it = options.begin(); it != options.end(); it++)
    {
        Option option = *it;
        // Differentiate between option with args and without
        if(option.parameter != "")
            count += 2;
        else
            count += 1;
    }

    return count;
}

bool Task::setShow_coordinate_system(bool value)
{
        printf("Hook called with: %s\n",value?"true":"false");

	//TODO Add your code here 
        if(!marsGraphics){
            fprintf(stderr,"Could not change view of coordinate systems without an Graphics interface\n");
            return false;
        }

  	//Call the base function, DO-NOT Remove
        if(value)
            marsGraphics->hideCoords();
        else
            marsGraphics->showCoords();

	return(mars::TaskBase::setShow_coordinate_system(value));
}

bool Task::setReaction_to_physics_error(::std::string const & value)
{
	//TODO Add your code here
        if(isConfigured()){
            if(!simulatorInterface){
                fprintf(stderr,"Task is not running could not set reaction to physics error");
                return false;
            }
            if(value == "abort" || value == "reset" || value == "warn" || value == "shutdown"){
                simulatorInterface->getControlCenter()->cfg->setPropertyValue("Simulator", "onPhysicsError","value", value);
            }else{
                fprintf(stderr,"Could not ser rection to physics: Possible Values: abort (killing sim), reset (ressing scene and mars), warn (keep mars running an print warnings), shutdown (stop physics but keep mars-running and set this tas to the error state)");
                return false;
            }
        }
        
        //Call the base function, DO-NOT Remove
	return(mars::TaskBase::setReaction_to_physics_error(value));
}

char** Task::setOptions(const std::vector<Option>& options)
{
    int count = getOptionCount(options)+ 1;
    char** argv = (char**) calloc(count, sizeof(char**));

    // Set executable name to mars_core
    count = 0;
    argv[count++] = "mars_core";

    std::vector<Option>::const_iterator it;
    for(it = options.begin(); it != options.end(); it++)
    {
        Option opt = *it;

        if(opt.name == "")
            continue;

        argv[count++] = strdup(opt.name.c_str());
        if(opt.parameter != "")
        {
            argv[count++] = strdup(opt.parameter.c_str());
        }
    }

    return argv;
}

bool Task::configureHook()
{
    if(_config_dir.get().empty())
    {
        RTT::log(RTT::Error) << "Config directory is not set! Cannot start mars." << RTT::endlog();
        throw std::runtime_error("Config directory is not set! Can not start mars");     
    }


    //check if the environemnt was sourced more than once and the path has more than one entry
    int pos = _config_dir.get().rfind(":/");
    if(pos != _config_dir.get().size()-1)
        _config_dir.set(_config_dir.get().substr(pos+1));
    
    RTT::log(RTT::Info) << "Calling configure: with " << _config_dir.get() << RTT::endlog();

    //mars is not setting the config path properly
    //therefore we have to go into to the config folder
    //if(0 != chdir(_config_dir.get().c_str()))
    //{
    //    RTT::log(RTT::Error) << "Config directory " << _config_dir.get() << " does not exist. Cannot start mars." << RTT::endlog();
    //    throw std::runtime_error(std::string("Config directory ") +_config_dir.get() +" does not exist. Can not start mars.");    
    //}

    // Startup of mars
    TaskArguments argument;
    argument.mars = this;
    argument.enable_gui = _enable_gui.get();
    argument.controller_port = _controller_port.get();
    argument.raw_options = _raw_options.get();
    argument.config_dir = _config_dir.get();
    argument.mars_property_list = _simulation_property_list.get();
    argument.initialized = false;
    argument.add_floor = _add_floor.get();
    argument.failed_to_init=false;
    argument.realtime_calc = _realtime_calc.get();

    int ret = pthread_create(&thread_info, NULL, startTaskFunc, &argument);
    if(ret)
    {
        RTT::log(RTT::Error) << "Failed to create MARS thread: pthread error " << ret << RTT::endlog();
        throw std::runtime_error("Failed to create MARS thread");
    }

    for(int i=0; !argument.initialized && !argument.failed_to_init;++i)
    {
        //give up after 10 sec
        if(i > 1000)
        {
            RTT::log(RTT::Error) << "Cannot start mars thread" << RTT::endlog();
            throw std::runtime_error("Cannot start mars thread!");
        }
        usleep(10000);
    }
    if(argument.failed_to_init){
            RTT::log(RTT::Error) << "Task failed to start, see Error above" << RTT::endlog();
            return false;
    }

    RTT::log(RTT::Info) << "Task running" << RTT::endlog();

    // Simulation is now up and running and plugins can be added
    // Configure basic functionality of mars
    // Check if distributed mars should be activated

    // todo: should be loaded via lib_manager
    /*
    if(_distributed_mars.get())
    {
        RTT::log(RTT::Info) << "Loading MultiSimPlugin" << RTT::endlog();
        multisimPlugin = new MultiSimPlugin(libManager);
        RTT::log(RTT::Info) << "MultiSimPlugin loaded" << RTT::endlog();
    }
    */
    if(!_initial_scene.get().empty()){
        printf("name: %s",_initial_scene.get().c_str());
        simulatorInterface->loadScene(_initial_scene.get(), std::string("initial"),true,true);
    }
    std::vector<std::string> sceneNames = _initial_scenes.get();
    if(!sceneNames.empty()){
		for (std::vector< std::string >::iterator scene = sceneNames.begin(); scene != sceneNames.end();scene++){
			printf("name: %s",scene->c_str());
			simulatorInterface->loadScene(*scene, *scene,true,true);
		}
    }
    

    std::vector<Positions> positions = _positions.get();
    if(!positions.empty()){
        for (std::vector< Positions >::iterator offset = positions.begin(); offset != positions.end();offset++){
            move_node(*offset);
        }
    }


    mars::Pose initial_pose = _initial_pose.get();
    if(!initial_pose.empty()){
    	mars::interfaces::ControlCenter* control = simulatorInterface->getControlCenter();
    	if (control){
			for (mars::Pose::iterator pos = initial_pose.begin(); pos != initial_pose.end();pos++){
				int marsMotorId = control->motors->getID( pos->name );
				mars::sim::SimMotor *motor = control->motors->getSimMotor( marsMotorId );
				if (motor){
					motor->setValue( pos->pos );
				}else{
					printf("no motor %s",pos->name.c_str());
				}
			}
    	}else{
    		printf("no contol center");
    	}
    }


    {//Setting the Step-with for the mars
    cfg_manager::cfgPropertyStruct c = simulatorInterface->getControlCenter()->cfg->getOrCreateProperty("Simulator", "calc_ms", _sim_step_size.get()*1000.0);
    c.dValue = _sim_step_size.get()*1000.0;
    simulatorInterface->getControlCenter()->cfg->setProperty(c);
    }


    {
    std::string value = _reaction_to_physics_error.get();
    if(value == "abort" || value == "reset" || value == "warn" || value == "shutdown"){
        simulatorInterface->getControlCenter()->cfg->setPropertyValue("Simulator", "onPhysicsError","value", value);
    }else{
        fprintf(stderr,"Wront selection for physic error setting\n");
        return false;
    }
    }
    
    simulatorInterface->getControlCenter()->cfg->setPropertyValue("Simulator","getTime:useNow","value",_use_now_instead_of_sim_time.get());

    setGravity_internal(_gravity.get());

    return updateDynamicProperties();
}


bool Task::startHook()
{
    // Simulation should be either started manually, 
    // or by using the control_action input_port
    //
    if (_start_sim.get()){
    	simulatorInterface->StartSimulation();
    }
    return true;
}

void Task::updateHook()
{
    mars::Control controlAction;
    if(_control_action.read(controlAction) == RTT::NewData)
    {
        switch(controlAction)
        {
            case START:
                RTT::log(RTT::Info) << "ControlAction: Start received" << RTT::endlog();
                if(!simulatorInterface->isSimRunning())
                    simulatorInterface->startStopTrigger();
                break;
            case PAUSE:
                RTT::log(RTT::Info) << "ControlAction: Pause received" << RTT::endlog();
                if(simulatorInterface->isSimRunning())
                    simulatorInterface->startStopTrigger();
                break;
            case RESET:
                RTT::log(RTT::Info) << "ControlAction: Reset received" << RTT::endlog();
                simulatorInterface->resetSim();
                break;
            case STEP:
                RTT::log(RTT::Info) << "ControlAction: Step received" << RTT::endlog();
                simulatorInterface->singleStep();
                break;
            default:
                RTT::log(RTT::Warning) << "Simulation: Unknown control action " << controlAction << " received" << RTT::endlog();

        }
    }

    if(simulatorInterface->hasSimFault()){
        std::cerr << "Simulation detected a Physics error, stopping all plugins and go to Exception state" << std::endl;
        for(unsigned int i=0;i<plugins.size();i++){
            plugins[i]->handleMarsShudown();
        }
        exception(PHYSICS_ERROR);
 //       QCoreApplication::quit(); //Quitting QApplication too
    }
}

void Task::errorHook()
{
    std::cout << "ERROR HOOK" << std::endl;
}

void Task::stopHook()
{
    /*
    std::cout << "STOP HOOK" << std::endl;
    for(unsigned int i=0;i<plugins.size();i++){
        plugins[i]->handleTaskShudown();
    }
    simulatorInterface->exitTask();

    std::cout << "STOP HOOK quitting qapp" << std::endl;
    QCoreApplication::quit(); //Quitting QApplication too
    std::cout << "STOP HOOK quitting qapp finish" << std::endl;
    */
}
        
void Task::registerPlugin(Plugin* plugin){
    plugins.push_back(plugin);
}

void Task::unregisterPlugin(Plugin* plugin){
    plugins.push_back(plugin);
}

void Task::cleanupHook()
{
    std::cout << "CLEANUP HOOK" << std::endl;
   

    for(unsigned int i=0;i<plugins.size();i++){
        plugins[i]->handleMarsShudown();
    }
    plugins.clear();

    simulatorInterface->exitMars();
    while( simulatorInterface->isSimRunning()) ;
    
    QCoreApplication::quit(); //Quitting QApplication too

    std::cout << "CLEANUP HOOK quitting qapp finish" << std::endl;

   // delete libManager;
    
//    libManager->releaseLibrary("mars_sim");
//    libManager->releaseLibrary("mars_gui");
//    libManager->releaseLibrary("mars_graphics");
//    libManager->releaseLibrary("gui_core");


 //   if(multisimPlugin) delete multisimPlugin;
}
/*
bool Task::recover(){
    std::cout << "RECOVER HOOK" << std::endl;
    return TaskBase::recover();
}
void Task::fatal(){
    std::cout << "FATAL HOOK" << std::endl;
    TaskBase::fatal();
}
void Task::exception(){
    std::cout << "EXCEPTION HOOK" << std::endl;
    TaskBase::exception();
}
*/

void Task::receiveData(
        const data_broker::DataInfo& info,
        const data_broker::DataPackage& package,
        int id) 
{
    _simulated_time.write(base::Time::fromMilliseconds(simulatorInterface->getTime()));
}

bool Task::setGravity_internal(::base::Vector3d const & value){
 simulatorInterface->setGravity(value);
}

bool Task::setGravity(::base::Vector3d const & value)
{
 if(!isConfigured()){
     //The configuration will be done within the configure hook later
     return(mars::TaskBase::setGravity(value));
 }
 
 setGravity_internal(value);
 return(mars::TaskBase::setGravity(value));
}

void Task::setPosition(::mars::Positions const & positions)
{
    if(isRunning() || isConfigured()){
        LOG_DEBUG("moving '%s' to %g/%g/%g\n", positions.nodename.c_str(), positions.posx, positions.posy, positions.posz);
        move_node(positions);
    }else{
        LOG_WARNING("setPosition called, but mars::Task is whether configured nor running ");
    }
    return;
}

bool Task::setSim_step_size(double value)
{
    //convert to ms
    value *= 1000.0;
    if(!isConfigured()){
        //The configuration will be done within the configure hook later
        return(mars::TaskBase::setSim_step_size(value));
    }
    cfg_manager::cfgPropertyStruct c = simulatorInterface->getControlCenter()->cfg->getOrCreateProperty("Simulator", "calc_ms", value);
    c.dValue = value;
    simulatorInterface->getControlCenter()->cfg->setProperty(c);
    return(mars::TaskBase::setSim_step_size(value));
}

void Task::move_node(::mars::Positions const & arg)
{
    mars::interfaces::NodeManagerInterface* nodes = simulatorInterface->getControlCenter()->nodes;
    mars::interfaces::NodeId id = nodes->getID(arg.nodename);
    if (id){
        mars::interfaces::NodeData nodedata = nodes->getFullNode(id);
        utils::Vector pos = nodes->getPosition(id);

        pos.x() = arg.posx;
        pos.y() = arg.posy;
        pos.z() = arg.posz;

        mars::utils::Quaternion rot = nodes->getRotation(id);

        mars::utils::Vector rotoff;

        rotoff.x() =  arg.rotx;
        rotoff.y() =  arg.roty;
        rotoff.z() =  arg.rotz;

        mars::utils::Quaternion newrot = mars::utils::eulerToQuaternion(rotoff);

        nodedata.pos = pos;
        nodedata.rot = newrot;

        nodes->editNode(&nodedata, mars::interfaces::EDIT_NODE_POS | mars::interfaces::EDIT_NODE_MOVE_ALL);
        nodes->editNode(&nodedata, mars::interfaces::EDIT_NODE_ROT | mars::interfaces::EDIT_NODE_MOVE_ALL);
    }else{
        LOG_WARN("node '%s' unknown\n", arg.nodename.c_str());
    }
}
