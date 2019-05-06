#ifndef MARS_WRENCH_TYPES_HH
#define MARS_WRENCH_TYPES_HH

#include <string>

namespace mars 
{

struct WrenchMapping {
    // Name of the mars sensor
    std::string sensor_name;

    // Index of the sensor value
    int sensor_index;

    // Map the measured force on this wrench element: [Fx|Fy|Fz|Tx|Ty|Tz]
    std::string ft_name;
};


}
#endif
