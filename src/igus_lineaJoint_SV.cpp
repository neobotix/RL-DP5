#include <cpr_robot.h>


namespace cpr_robot
{
    //! All model specific parameters, like gear ratios, position ranges, etc. are set during construction.
    igus_lineaJoint_SV::igus_lineaJoint_SV() :
        Robot(1,1)
    {   
        set_ModelName("igus_lineaJoint_SV");
        set_GearRatio(0,50.0);
        set_JointName(0,"arm_joint");
        set_JointType(0, true); // true - Prismatic joints, by default - revolute joint
        set_PulleyRadius(0, 1);
        set_TicksPerMotorRotation(0,2000);
        set_MaxVelocity(0,0.3);
        set_MinPosition(0,0.0);
        set_MaxPosition(0,350.0); // In meters
        set_MotorOffset(0,0);
        define_Output(true,0,0,"Digital out 1");
        define_Input(true,0,0,"Digital in 1");
    }

    //! Destructs an instance of the igus_lineaJoint_SV class  
    igus_lineaJoint_SV::~igus_lineaJoint_SV()
    {

    }

}