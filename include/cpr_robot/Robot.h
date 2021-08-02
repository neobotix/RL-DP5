#pragma once

#include <vector>

namespace cpr_robot
{
	//! \class Robot Robot.h <cpr_robot.h>
	//! \brief Abstract class representing a generic robot. 
	//!
	//! This class hold all information associated with a robot: joints, connection status and information about the specific model. It serves as base class for model specific implementations and handles publishing information on ROS topics and services as well as listening to ROS topics for commands.
    class Robot
    {
    private:
        //! \brief This struct holds information about a digital input or output channel: whether it is on a sperate IO-board or associated to a joint and the index of the module/joint.
        struct IOchannel
        {
        public:
            //! If set to true the I/O channel is associated with an element of m_pDigitalIOs, elsewise with an element of m_pJoints.
            bool OnSeparateModule;
            //! Index of the associated element in m_pDigitalIOs or m_pJoints.
            uint8_t ModuleIndex;
            //! The physical channel index on the corresponding module. A maximum of 8 channels per module is supported.
            uint8_t ChannelIndex;
            std::string Name;
        };
    public:
        //! Statusflag indicating that at least one of the modules that is controlling the robot is not connected.
        static constexpr uint32_t STATUSFLAG_DISCONNECTED=0x00000100;
        //! Command requesting that connection should be established with all modules controlling the robot.
        static constexpr uint32_t COMMAND_CONNECT=1;
        //! Command requesting that the connections to all modules controlling the robot should be closed.
        static constexpr uint32_t COMMAND_DISCONNECT=2;
        //! Command requesting that motor motion should be enabled for all joints.
        static constexpr uint32_t COMMAND_ENABLE=3;
        //! Command requesting that motor motion should be disabled for all joints.
        static constexpr uint32_t COMMAND_DISABLE=4;
        //! Command sending a new override value contolling the speed of the joints.
        static constexpr uint32_t COMMAND_OVERRIDE=5;
        //! Command requesting the referencing procedure to be initiated for all joints.
        static constexpr uint32_t COMMAND_STARTREFERENCING=6;
        //! Command requesting that the encoder positions of all joints should be reset.
        static constexpr uint32_t COMMAND_SETZERO=7;
        //! Command requesting that a digital output should be enabled.
        static constexpr uint32_t COMMAND_DOUT_ENABLE=8;
        //! Command requesting that a digital output should be disabled.
        static constexpr uint32_t COMMAND_DOUT_DISABLE=9;
    private:
        //! The number of joints of the robot.
        const size_t m_CountJoints;
        //! The number of digital I/O boards of the robot.
        const size_t m_CountIOmodules;
        //! A handle to the current ROS node.
        ros::NodeHandle m_Node;
        //! Publisher that will publish the state of all joints on the /joint_states ROS topic.
        ros::Publisher m_JointStatePublisher;
        //! Publisher that will publish the state of the robot (error flags, etc.) on the /RobotState ROS topic.
        ros::Publisher m_RobotStatePublisher;
        //! Publisher that will publish the states of the digital inputs on the /InputChannels ROS topic.
        ros::Publisher m_InputChannelsPublisher;
        //! Publisher that will publish the states of the digital outputs on the /OutputChannels ROS topic.
        ros::Publisher m_OutputChannelsPublisher;
        //! Provides information about the robot (number of joints, model deisgnation) over the /GetRobotInfo ROS service.
        ros::ServiceServer m_GetRobotInfoServer;
        //! Provides information about a a specific joint (name, type) over the /GetJointInfo ROS service.
        ros::ServiceServer m_GetJointInfoServer;
        //! Allows the robot to receive commands over the /RobotCommand ROS service.
        ros::ServiceServer m_RobotCommandServer;
        //! Instance of the Bus class that will be used to communicate with the firmware of the modules that are controlling the robot.
        Bus m_Bus;
        //! Pointer to an array of instances of the Joint class. One entry per joint.
        Joint** m_pJoints;
        //! Pointer to an array of instances of the Joint class. One entry per digital I/O module.
        Joint** m_pIOmodules;
        //! The model designation of the robot.
        std::string m_ModelName;
        //! The Joint type.
        std::string m_JointType;
        //! flag variable.
        bool m_LinearActuator = false;
        //! The current override value used to control the speed of the joints.
        double m_Override;
        //! Vector containing all defined output channels.
        std::vector<IOchannel> m_OutputChannels;
        //! Vector containing all defined input channels.
        std::vector<IOchannel> m_InputChannels;
        bool GetRobotInfoHandler(cpr_robot::GetRobotInfo::Request  &req, cpr_robot::GetRobotInfo::Response &res);
        bool GetJointInfoHandler(cpr_robot::GetJointInfo::Request  &req, cpr_robot::GetJointInfo::Response &res);
        bool RobotCommandHandler(cpr_robot::RobotCommand::Request  &req, cpr_robot::RobotCommand::Response &res);
    protected:
        uint32_t define_Input(const bool onSeperateModule, const uint8_t moduleIndex, const uint8_t channelIndex, const std::string& name);
        uint32_t define_Output(const bool onSeperateModule, const uint8_t moduleIndex, const uint8_t channelIndex, const std::string& name);
        void set_Override(const double override);
        double get_Override() const;
        void set_ModelName(const std::string& name);
        const std::string& get_ModelName();
        void set_JointName(const size_t jointId, const std::string& name);
        void set_JointType(const size_t jointId, bool linear_actuator);
        const std::string& get_JointName(const size_t jointId);
        void set_GearRatio(const size_t jointId, const double ratio);
        double get_GearRatio(const size_t jointId);
        void set_MaxVelocity(const size_t jointId, const double velocity);
        double get_MaxVelocity(const size_t jointId);
        void set_TicksPerMotorRotation(const size_t jointId, const int32_t ticks);
        int32_t get_TicksPerMotorRotation(const size_t jointId);
        double get_MaxPosition(const size_t jointId) const;
        void set_MaxPosition(const size_t jointId, const double position);
        double get_MinPosition(const size_t jointId) const;
        void set_MinPosition(const size_t jointId, const double position);
        void set_MotorOffset(const size_t jointId, const int32_t ticks);
        void set_PulleyRadius(const size_t jointId, double radius);
        int32_t get_MotorOffset(const size_t jointId);
        Robot(const size_t countJoints, const size_t countDigitalIOs);
        virtual void OnInit();
        void set_Output(const uint32_t index, const bool state);
        bool get_Output(const uint32_t index) const;
        bool get_Input(const uint32_t index) const;
    public:
        void Init();
        void Read();
        void Write();
        void PublishState();
        virtual ~Robot();
    };
}