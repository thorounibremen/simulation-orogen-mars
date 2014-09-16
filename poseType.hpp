#ifndef POSE_TYPE_HH
#define POSE_TYPE_HH

#include <vector>
#include <string>

namespace simulation
{

struct JointPos{
		std::string name;
		float pos;
	};

//struct Pose {
//	std::vector<JointPos> pose;
//};

typedef std::vector<JointPos> Pose;

}
#endif
