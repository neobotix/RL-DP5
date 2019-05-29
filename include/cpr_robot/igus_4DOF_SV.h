#pragma once

namespace cpr_robot
{
	//! \class igus_4DOF_SV igus_4DOF_SV.h <cpr_robot.h>
	//! \brief Class representing a robolink 4DOF small version robot from igus. 
	//!
	//! All model specific parameters are handled by this class
    class igus_4DOF_SV : public Robot
    {
        public:
            igus_4DOF_SV();
            virtual ~igus_4DOF_SV();
    };
}