#include <cpr_robot.h>
#include <iostream>
#include <sstream>

namespace cpr_robot
{
    //! \brief Callback function handling requests to the /GetRobotInfo ROS service.
    //! \param req The request to be handled.
    //! \param res The answer that will be returned to the request.
    //! \return Returns true if the request has been handled successfully, elsewise returns false.
    bool Robot::GetRobotInfoHandler(cpr_robot::GetRobotInfo::Request  &req, cpr_robot::GetRobotInfo::Response &res)
    {
        res.Model=get_ModelName();
        res.CountJoints=(uint32_t)m_CountJoints;
        for(size_t i=0;i<m_InputChannels.size();i++)
            res.InputChannels.push_back(m_InputChannels[i].Name);
        for(size_t i=0;i<m_OutputChannels.size();i++)
            res.OutputChannels.push_back(m_OutputChannels[i].Name);
         return true;
    }

    //! \brief Callback function handling requests to the /GetJointInfo ROS service.
    //! \param req The request to be handled.
    //! \param res The answer that will be returned to the request.
    //! \return Returns true if the request has been handled successfully, elsewise returns false.
    bool Robot::GetJointInfoHandler(cpr_robot::GetJointInfo::Request  &req, cpr_robot::GetJointInfo::Response &res)
    {
        if(req.JointId<m_CountJoints)
        {
            res.JointName=m_pJoints[req.JointId]->get_JointName();
            res.JointType=0;
            return true;
        }
        else
        {
            return false;
        }
        
    }

    //! \brief Sets the model designation of the robot. Should be called before the Init method is called.
    //! \param name The model designation of the robot.
    void Robot::set_ModelName(const std::string& name)
    {
        m_ModelName=name;
    }
        
    //! \brief Gets the model designation of the robot.
    //! \return The model designation of the robot.
    const std::string& Robot::get_ModelName()
    {
        return m_ModelName;
    }

    //! \brief Constructs an instance of the Robot class.
    //! \param countJoints The number of joints of the robot.
    //! \param countDigitalIOs The number of I/O modules of the robot.
    Robot::Robot(const size_t countJoints, const size_t countIOmodules) :
        m_CountJoints(countJoints),
        m_CountIOmodules(countIOmodules),
        m_Bus("can0")
    {
        m_ModelName="Unknown Model";
        m_pJoints=new Joint*[m_CountJoints];
        for(size_t i=0;i<m_CountJoints;i++)
            m_pJoints[i]=new Joint(m_Bus,(unsigned int)(i+1));
        m_pIOmodules=new Joint*[m_CountIOmodules];
        for(size_t i=0;i<m_CountIOmodules;i++)
            m_pIOmodules[i]=new Joint(m_Bus,(unsigned int)(i+7));
        m_JointStatePublisher=m_Node.advertise<sensor_msgs::JointState>("/joint_states", 50);
        m_RobotStatePublisher=m_Node.advertise<cpr_robot::RobotState>("/robot_state",50);
        m_InputChannelsPublisher=m_Node.advertise<cpr_robot::ChannelStates>("/InputChannels",50);
        m_OutputChannelsPublisher=m_Node.advertise<cpr_robot::ChannelStates>("/OutputChannels",50);
        m_GetRobotInfoServer=m_Node.advertiseService("/GetRobotInfo",&Robot::GetRobotInfoHandler, this);
        m_GetJointInfoServer=m_Node.advertiseService("/GetJointInfo",&Robot::GetJointInfoHandler, this);
        m_RobotCommandServer=m_Node.advertiseService("/RobotCommand",&Robot::RobotCommandHandler, this);
        m_Override=0.25;
    }

    //! \brief Callback function handling requests to the /RobotCommand ROS service.
    //! \param req The request to be handled.
    //! \param res The answer that will be returned to the request.
    //! \return Returns true if the request has been handled successfully, elsewise returns false.
    bool Robot::RobotCommandHandler(cpr_robot::RobotCommand::Request  &req, cpr_robot::RobotCommand::Response &res)
    {
        switch(req.CommandId)
        {
            default:
                ROS_ERROR("Received invalid command from %s: %ui",req.Sender.c_str(),req.CommandId);
                return false;
            case COMMAND_SETZERO:
                ROS_INFO("COMMAND_SETZERO from %s.",req.Sender.c_str());
                for(size_t i=0;i<m_CountJoints;i++)
                    m_pJoints[i]->SetZero();
                for(size_t i=0;i<m_CountIOmodules;i++)
                    m_pIOmodules[i]->SetZero();
                return true;           
            case COMMAND_OVERRIDE:
                ROS_INFO("COMMAND_OVERRIDE from %s. Override=%lf%%",req.Sender.c_str(),100.0*req.PayloadFloat);
                m_Override=req.PayloadFloat;
                return true;
            case COMMAND_CONNECT:
                ROS_INFO("COMMAND_CONNECT from %s.",req.Sender.c_str());
                for(uint32_t i=0;i<m_CountJoints;i++)
                    m_pJoints[i]->Start();
                for(size_t i=0;i<m_CountIOmodules;i++)
                    m_pIOmodules[i]->Start();
                m_Bus.Start();
                return true;
            case COMMAND_DISCONNECT:
                ROS_INFO("COMMAND_DISCONNECT from %s.",req.Sender.c_str());
                for(uint32_t i=0;i<m_CountJoints;i++)
                    m_pJoints[i]->Stop();
                for(size_t i=0;i<m_CountIOmodules;i++)
                    m_pIOmodules[i]->Stop();
                m_Bus.Stop();
                return true;
            case COMMAND_ENABLE:
                ROS_INFO("COMMAND_ENABLE from %s.",req.Sender.c_str());
                for(size_t i=0;i<m_CountJoints;i++)
                    m_pJoints[i]->EnableMotor();
                for(size_t i=0;i<m_CountIOmodules;i++)
                    m_pIOmodules[i]->EnableMotor();
                return true;
            case COMMAND_DISABLE:
                ROS_INFO("COMMAND_DISABLE from %s.",req.Sender.c_str());
                for(size_t i=0;i<m_CountJoints;i++)
                    m_pJoints[i]->DisableMotor();
                for(size_t i=0;i<m_CountIOmodules;i++)
                    m_pIOmodules[i]->DisableMotor();
                return true;
            case COMMAND_STARTREFERENCING:
                ROS_INFO("COMMAND_STARTREFERENCING from %s.",req.Sender.c_str());
                for(size_t i=0;i<m_CountJoints;i++)
                    m_pJoints[i]->StartReferencing();
                return true;
            case COMMAND_DOUT_ENABLE:
                ROS_INFO("COMMAND_DOUT_ENABLE from %s.",req.Sender.c_str());
                if((req.PayloadInt>=0)&&(req.PayloadInt<m_OutputChannels.size()))
                {
                    set_Output((uint32_t)req.PayloadInt,true);
                    return true;
                }
                else
                {
                    ROS_ERROR("Received invalid digital output: %li",req.PayloadInt);
                    return false;
                }
            case COMMAND_DOUT_DISABLE:
                ROS_INFO("COMMAND_DOUT_DISABLE from %s.",req.Sender.c_str());
                if((req.PayloadInt>=0)&&(req.PayloadInt<m_OutputChannels.size()))
                {
                    set_Output((uint32_t)req.PayloadInt,false);
                    return true;
                }
                else
                {
                    ROS_ERROR("Received invalid digital output: %li",req.PayloadInt);
                    return false;
                }
        }
    }

    //! \brief Destructor of the Robot class.
    Robot::~Robot()
    {
        for(size_t i=0;i<m_CountIOmodules;i++)
            delete m_pIOmodules[i];
        delete[] m_pIOmodules;
        for(size_t i=0;i<m_CountJoints;i++)
            delete m_pJoints[i];
        delete[] m_pJoints;
    }
    
    //! \brief Sets the override value used to control the speed of the joints.
    //! \param override The new override value. Must be between 0 and 1.
    void Robot::set_Override(const double override)
    {
        m_Override=override;
    }
    
    //! \brief Gets the override value used to control the speed of the joints.
    //! \return The current override value.
    double Robot::get_Override() const
    {
        return m_Override;
    }

    //! \brief Publishes the current state of the robot on the /joint_states and /RobotState ROS topics.
    void Robot::PublishState()
    {
        sensor_msgs::JointState joint_states;
        joint_states.header.stamp = ros::Time::now();
        m_JointStatePublisher.publish(joint_states);
        for(size_t i=0;i<m_CountJoints;i++)
            m_pJoints[i]->PublishState(joint_states);
        m_JointStatePublisher.publish(joint_states);
        cpr_robot::ChannelStates inputChannels;
        inputChannels.Header.stamp = ros::Time::now();
        for(size_t i=0;i<m_InputChannels.size();i++)
            inputChannels.state.push_back(get_Input(i));
        m_InputChannelsPublisher.publish(inputChannels);
        cpr_robot::ChannelStates outputChannels;
        outputChannels.Header.stamp = ros::Time::now();
        for(size_t i=0;i<m_OutputChannels.size();i++)
            outputChannels.state.push_back(get_Output(i));
        m_OutputChannelsPublisher.publish(outputChannels);
        cpr_robot::RobotState robot_state;
        robot_state.Header.stamp=ros::Time::now();
        robot_state.Override=m_Override;
        robot_state.StatusFlags=0;
        for(size_t i=0;i<m_CountJoints;i++)
        {
            robot_state.StatusFlags|=(uint32_t)m_pJoints[i]->get_ErrorFlags();
            robot_state.StatusFlags|=((uint32_t)(m_pJoints[i]->get_IsReferenced()?1:0))<<(16+i);
        }
        if(!m_Bus.get_IsConnected())
            robot_state.StatusFlags|=STATUSFLAG_DISCONNECTED;
        m_RobotStatePublisher.publish(robot_state);
    }

    //! \brief Gets the upper bound for the position of a specific joint.
    //! \param jointId The ID of the joint for wich the upper bound is to be retrieved.
    //! \return The current maximum allowed position of the joint. Measured in degrees.
    double Robot::get_MaxPosition(const size_t jointId) const
    {
        assert(jointId<m_CountJoints);
        return m_pJoints[jointId]->get_MaxPosition();
    }

    //! \brief Sets the upper bound for the position of a specific joint.
    //! \param jointId The ID of the joint for which the upper bound is to be set.
    //! \param position The new maximum allowed position of the joint. Measured in degrees.
    void Robot::set_MaxPosition(const size_t jointId, const double position)
    {
        assert(jointId<m_CountJoints);
        m_pJoints[jointId]->set_MaxPosition(position);
    }

    //! \brief Gets the lower bound for the position of a specific joint.
    //! \param jointId The ID of the joint for wich the lower bound is to be retrieved.
    //! \return The current minimum allowed position of the joint. Measured in degrees.
    double Robot::get_MinPosition(const size_t jointId) const
    {
        assert(jointId<m_CountJoints);
        return m_pJoints[jointId]->get_MinPosition();
    }


    //! \brief Sets the lower bound for the position of a specific joint.
    //! \param jointId The ID of the joint for which the lower bound is to be set.
    //! \param position The new minimum allowed position of the joint. Measured in degrees.
    void Robot::set_MinPosition(const size_t jointId, const double position)
    {
        assert(jointId<m_CountJoints);
        m_pJoints[jointId]->set_MinPosition(position);
    }
    
    //! \brief Sets the zero position of the motor of a specific joint.
    //! \param jointId The ID of the joint for which the zero position is to be set.
    //! \param ticks The new zero offset of the motor measured in encoder ticks.
    void Robot::set_MotorOffset(const size_t jointId, const int32_t ticks)
    {
        assert(jointId<m_CountJoints);
        m_pJoints[jointId]->set_MotorOffset(ticks);
    }
    
    //! \brief Gets the zero position of the motor of a specific joint.
    //! \param jointId The ID of the joint for which the zero position is to be retrieved.
    //! \return The current zero offset of the motor measured in encoder ticks.
    int32_t Robot::get_MotorOffset(const size_t jointId)
    {
        assert(jointId<m_CountJoints);
        return m_pJoints[jointId]->get_MotorOffset();
    }

    //! \brief Reads the current state of the robot from the firmware.
    void Robot::Read()
    {
        for(size_t i=0;i<m_CountJoints;i++)
            m_pJoints[i]->Read();
    }

    //! \brief Sends the current motion commands to the firmware in the modules that are controlling the motors of the robot.
    void Robot::Write()
    {
        for(size_t i=0;i<m_CountJoints;i++)
            m_pJoints[i]->Write(m_Override);
    }

    //! \brief Initializes the robot. 
    //! Should be called after all robot parameters (gear ratios, ticks per motor rotation) have been set. Will call the virtual OnInit method.
    void Robot::Init()
    {
        OnInit();
    }

    //! \brief Initializes the robot. 
    //!
    //! Should be called after all robot parameters (gear ratios, ticks per motor rotation) have been set.
    void Robot::OnInit()
    {
        for(size_t i=0;i<m_CountJoints;i++)
            m_pJoints[i]->Init();
        for(size_t i=0;i<m_CountIOmodules;i++)
            m_pIOmodules[i]->Init();
    }

    //! \brief Sets the name of a specific joint that will be used for communication over ROS topics and services.
    //! \param jointId The ID of the joint for which the name should be set.
    //! \param name The desired name of the joint.
    void Robot::set_JointName(const size_t jointId, const std::string& name)
    {
        assert(jointId<m_CountJoints);
        m_pJoints[jointId]->set_JointName(name);
    }

    //! \brief Gets the name of a specific joint that is used for communication over ROS topics and services.
    //! \param jointId The ID of the joint whos name is to be retrieved.
    //! \return The name of the joint.
    const std::string& Robot::get_JointName(const size_t jointId)
    {
        assert(jointId<m_CountJoints);
        return m_pJoints[jointId]->get_JointName();
    }

    //! \brief Sets the gear ratio of a specific joint.
    //! \param jointId The ID of the joint for which the gear ratio is to be set.
    //! \param ratio The gear ratio of for the joint.
    void Robot::set_GearRatio(const size_t jointId, const double ratio)
    {
        assert(jointId<m_CountJoints);
        m_pJoints[jointId]->set_GearRatio(ratio);
    }

    //! \brief Gets the gear ratio of a specific joint.
    //! \param jointId The ID of the joint for which the gear ratio is to be retrieved.
    //! \return The gear ratio of the joint.
    double Robot::get_GearRatio(const size_t jointId)
    {
        assert(jointId<m_CountJoints);
        return m_pJoints[jointId]->get_GearRatio();
    }

    //! \brief Sets the number of encoder ticks representic one rotation of the motor for a specific joint.
    //! \param jointId The ID of the joint for which the ticks per rotation is to be set.
    //! \param ticks The number of encoder ticks representing exactly one rotation of the motor.
    void Robot::set_TicksPerMotorRotation(const size_t jointId, const int32_t ticks)
    {
        assert(jointId<m_CountJoints);
        m_pJoints[jointId]->set_TicksPerMotorRotation(ticks);
    }

    //! \brief Gets the number of encoder ticks representic one rotation of the motor for a specific joint.
    //! \param jointId The ID of the joint for which the ticks per rotation is to be retrieved.
    //! \return The number of encoder ticks representing exactly one rotation of the motor.
    int32_t Robot::get_TicksPerMotorRotation(const size_t jointId)
    {
        assert(jointId<m_CountJoints);
        return m_pJoints[jointId]->get_TicksPerMotorRotation();
    }

    //! \brief Sets the maximal allowed angular velocity of a specific joint.
    //! \param jointId The ID of the joint for which the maximal angular velocity is to be set.
    //! \param velocity The maximal angular velocity measured in degrees per second.
    void Robot::set_MaxVelocity(const size_t jointId, const double velocity)
    {
        assert(jointId<m_CountJoints);
        m_pJoints[jointId]->set_MaxVelocity(velocity);
    }

    //! \brief Gets the maximal allowed angular velocity of a specific joint.
    //! \param jointId The ID of the joint for which the maximal angular velocity is to be retrieved.
    //! \return The maximal angular velocity measured in degrees per second.
    double Robot::get_MaxVelocity(const size_t jointId)
    {
        assert(jointId<m_CountJoints);
        return m_pJoints[jointId]->get_MaxVeclocity();
    }

    //! \brief Define and name a new input channel
    //! \param onSeperateModule Set to true if the channel is controlled from a DIN-rail module not associated with a joint.
    //! \param moduleIndex Index of the joint or DIN-rail module controlling the digital input.
    //! \param channelIndex Index of the channel on the respective control module.
    //! \param name The name that should be given to the input channel.
    //! \return The ID of the newly defined digital input.
    uint32_t Robot::define_Input(const bool onSeperateModule, const uint8_t moduleIndex, const uint8_t channelIndex, const std::string& name)
    {
        assert(onSeperateModule?moduleIndex<m_CountIOmodules:moduleIndex<m_CountJoints);
        assert(channelIndex<7);
        uint32_t id=(uint32_t)m_InputChannels.size();
        IOchannel channel;
        channel.OnSeparateModule=onSeperateModule;
        channel.ModuleIndex=moduleIndex;
        channel.ChannelIndex=channelIndex;
        channel.Name=name;
        m_InputChannels.push_back(channel);
        return id;
    }

    //! \brief Define and name a new output channel
    //! \param onSeperateModule Set to true if the channel is controlled from a DIN-rail module not associated with a joint.
    //! \param moduleIndex Index of the joint or DIN-rail module controlling the digital output.
    //! \param channelIndex Index of the channel on the respective control module.
    //! \param name The name that should be given to the output channel.
    //! \return The ID of the newly defined digital output.
    uint32_t Robot::define_Output(const bool onSeperateModule, const uint8_t moduleIndex, const uint8_t channelIndex, const std::string& name)
    {
        assert(onSeperateModule?moduleIndex<m_CountIOmodules:moduleIndex<m_CountJoints);
        assert(channelIndex<7);
        uint32_t id=(uint32_t)m_OutputChannels.size();
        IOchannel channel;
        channel.OnSeparateModule=onSeperateModule;
        channel.ModuleIndex=moduleIndex;
        channel.ChannelIndex=channelIndex;
        channel.Name=name;
        m_OutputChannels.push_back(channel);
        return id;
    }
        
    //! \brief Set the state of a digital output.
    //! \param index The ID of the digital output.
    //! \param state The new state of the digital output.
    void Robot::set_Output(const uint32_t index, const bool state)
    {
        assert(index<m_OutputChannels.size());
        if(m_OutputChannels[index].OnSeparateModule)
            m_pIOmodules[m_OutputChannels[index].ModuleIndex]->set_DigitalOutput(m_OutputChannels[index].ChannelIndex,state);
        else
            m_pJoints[m_OutputChannels[index].ModuleIndex]->set_DigitalOutput(m_OutputChannels[index].ChannelIndex,state);
    }
    
    //! \brief Get the state of a digital input.
    //! \param index The ID of the digital input.
    //! \return The current state of the digital input.
    bool Robot::get_Input(const uint32_t index) const
    {
        assert(index<m_InputChannels.size());
        if(m_InputChannels[index].OnSeparateModule)
            return m_pIOmodules[m_InputChannels[index].ModuleIndex]->get_DigitalInput(m_InputChannels[index].ChannelIndex);
        else
            return m_pJoints[m_InputChannels[index].ModuleIndex]->get_DigitalInput(m_InputChannels[index].ChannelIndex);
    }
    
    //! \brief Get the state of a digital output.
    //! \param index The ID of the digital output.
    //! \return The current state of the digital output.
    bool Robot::get_Output(const uint32_t index) const
    {
        assert(index<m_OutputChannels.size());
        if(m_OutputChannels[index].OnSeparateModule)
            return m_pIOmodules[m_OutputChannels[index].ModuleIndex]->get_DigitalOutput(m_OutputChannels[index].ChannelIndex);
        else
            return m_pJoints[m_OutputChannels[index].ModuleIndex]->get_DigitalOutput(m_OutputChannels[index].ChannelIndex);
    }


}