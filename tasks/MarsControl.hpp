#ifndef SIMULATION_MARS_CONTROL_HPP
#define SIMULATION_MARS_CONTROL_HPP

#ifndef __orogen
#include <string>
#endif

namespace mars {

   enum Control { NONE = 0, START, PAUSE, RESET, STEP };

   struct Option
   {
       std::string name;
       std::string parameter;

       #ifndef __orogen
       Option() : name(), parameter() {}
       Option(const std::string& _name, const std::string& _parameter) : name(_name), parameter(_parameter){ }
       #endif

   };

   /**
    * position offset parameter
    */
   struct Positions
   {
	   std::string nodename;
	   float posx,posy,posz;
	   float rotx,roty,rotz;
   };

    struct SimulationProperty
    {
        std::string lib_name;
        std::string property_name;
        std::string value;
    };

}
#endif // SIMULATION_MARS_CONTROL_HPP
