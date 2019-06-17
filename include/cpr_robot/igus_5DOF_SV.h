#pragma once

namespace cpr_robot
{
	//! \class igus_5DOF_SV igus_5DOF_SV.h <cpr_robot.h>
	//! \brief Class representing a robolink 5DOF small version robot from igus. 
	//!
	//! All model specific parameters are handled by this class
    class igus_5DOF_SV : public Robot
    {
        public:
            igus_5DOF_SV();
            virtual ~igus_5DOF_SV();
    };
}