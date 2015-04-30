#ifndef MARS_JOINT_TYPES_HH
#define MARS_JOINT_TYPES_HH

#include <string>
#include <base/NamedVector.hpp>
#include <base/Time.hpp>


namespace mars 
{

struct ParallelKinematic{
	std::string externalName;
	std::string internalName1;
	std::string internalName2;
};

struct JointCurrents: public base::NamedVector< float >{
	base::Time time;
};


}
#endif
