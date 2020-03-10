#include <cpr_robot.h>


namespace cpr_robot
{
    //! All model specific parameters, like gear ratios, position ranges, etc. are set during construction.
    igus_5DOF_SV::igus_5DOF_SV() :
        Robot(5,1)
    {
        set_ModelName("igus_5DOF_SV");
        set_GearRatio(0,50.0);
        set_GearRatio(1,50.0);
        set_GearRatio(2,-38.0);
        set_GearRatio(3,-38.0);
        set_GearRatio(4,-28.0);
        set_JointName(0,"joint1");
        set_JointName(1,"joint2");
        set_JointName(2,"joint3");
        set_JointName(3,"joint4");
        set_JointName(4,"joint5");
        set_TicksPerMotorRotation(0,2000);
        set_TicksPerMotorRotation(1,2000);
        set_TicksPerMotorRotation(2,2000);
        set_TicksPerMotorRotation(3,2000);
        set_TicksPerMotorRotation(4,2000);
        set_MaxVelocity(0,45.0);
        set_MaxVelocity(1,45.0);
        set_MaxVelocity(2,45.0);
        set_MaxVelocity(3,60.0);
        set_MaxVelocity(4,60.0);
        set_MinPosition(0,-140.0);
        set_MaxPosition(0,140.0);
        set_MinPosition(1,-45.0);
        set_MaxPosition(1,70.0);
        set_MinPosition(2,-90.0);
        set_MaxPosition(2,60.0);
        set_MinPosition(3,-30.0);
        set_MaxPosition(3,120.0);
        set_MinPosition(4,-170.0);
        set_MaxPosition(4,170.0);
        set_MotorOffset(0,0);
        set_MotorOffset(1,0);
        set_MotorOffset(2,0);
        set_MotorOffset(3,0);
        set_MotorOffset(4,0);
        define_Output(true,0,0,"Digital out 1");
        define_Output(true,0,1,"Digital out 2");
        define_Output(true,0,2,"Digital out 3");
        define_Output(true,0,3,"Digital out 4");
        define_Output(true,0,4,"Digital out 5");
        define_Output(true,0,5,"Digital out 6");
        define_Output(true,0,6,"Digital out 7");
        define_Input(true,0,0,"Digital in 1");
        define_Input(true,0,1,"Digital in 2");
        define_Input(true,0,2,"Digital in 3");
        define_Input(true,0,3,"Digital in 4");
        define_Input(true,0,4,"Digital in 5");
        define_Input(true,0,5,"Digital in 6");
        define_Input(true,0,6,"Digital in 7");
    }

    //! Destructs an instance of the igus_5DOF_SV class  
    igus_5DOF_SV::~igus_5DOF_SV()
    {

    }

}