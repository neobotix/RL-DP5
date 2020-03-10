#include "cpr_robot.h"
#include "qt_includes.h"
#include "TaggedButton.h"
#include "JointControl.h"
#include <rviz/panel.h>
#include "RobotPanel.h"
#include <sstream>

namespace cpr_rviz
{
    //! \brief Constructor of the RobotPanel class.
    //! \param parent The parent widget.
    RobotPanel::RobotPanel( QWidget* parent ) :
        rviz::Panel( parent ),
        m_ConnectButton(this),
        m_EnableButton(this),
        m_InputGroupBox(this),
        m_OutputGroupBox(this),
        m_ReferenceButton(this),
        m_ZeroButton(this),
        m_OverrideSlider(Qt::Horizontal,this),
        m_OverrideLabel(this),
        m_OverrideSliderValid(false)
    {
        InitializeROS();
        InitializeState();
        InitializeUI();
    }

    //! \brief Destructor of the RobotPanel class.
    RobotPanel::~RobotPanel()
    {
        m_ControlButtonsLayout.setParent(nullptr);
        m_JointsLayout.setParent(nullptr);
        for(size_t i=0;i<m_Inputs.size();i++)
        {
            delete m_pInputs[i];
        }
        for(size_t i=0;i<m_Outputs.size();i++)
        {
            delete m_pOutputs[i];
        }
        delete[] m_pOutputs;
        for(uint32_t i=0;i<m_CountJoints;i++)
        {
            delete m_pJointControls[i];
        }
        delete[] m_pJointControls;
    }

    //! \brief Initializes all relevant UI elements.
    void RobotPanel::InitializeUI()
    {
        m_ModelNameLabel.setText(m_ModelName.c_str());
        m_MainLayout.addRow(tr("Model:"), &m_ModelNameLabel);
        m_ControlButtonsLayout.addWidget(&m_ConnectButton);
        m_ControlButtonsLayout.addWidget(&m_EnableButton);
        m_ControlButtonsLayout.addWidget(&m_ReferenceButton);
        m_ControlButtonsLayout.addWidget(&m_ZeroButton);
        m_ReferenceButton.setText("Reference");
        m_ZeroButton.setText("Set Zero");
        double k=0.5*(double)m_CountJoints;
        int rows=(int)round(k+0.5);
        m_MainLayout.addRow(&m_ControlButtonsLayout);
        m_MainLayout.addRow(tr("Override:"), &m_OverrideLabel);
        m_OverrideSlider.setMinimum(0);
        m_OverrideSlider.setMaximum(100);
        m_OverrideSlider.setTickInterval(10);
        m_OverrideSlider.setTickPosition(QSlider::TicksBelow);
        m_MainLayout.addRow(&m_OverrideSlider);
        m_pInputs=new QCheckBox*[m_Inputs.size()];
        for(size_t i=0;i<m_Inputs.size();i++)
        {
            m_pInputs[i]=new QCheckBox(&m_InputGroupBox);
            m_pInputs[i]->setText(m_Inputs[i].c_str());
            m_InputLayout.addWidget(m_pInputs[i],i/4,i%4);
        }
        m_InputGroupBox.setLayout(&m_InputLayout);
        m_InputGroupBox.setTitle("Digital inputs:");
        m_MainLayout.addRow(&m_InputGroupBox);
        m_pOutputs=new TaggedButton*[m_Outputs.size()];
        for(size_t i=0;i<m_Outputs.size();i++)
        {
            m_pOutputs[i]=new TaggedButton((int)i,&m_OutputGroupBox);
            m_pOutputs[i]->setText(m_Outputs[i].c_str());
            m_OutputLayout.addWidget(m_pOutputs[i],i/4,i%4);
            m_pOutputs[i]->setCheckable(true);
            connect(m_pOutputs[i],&TaggedButton::buttonClicked,this,&RobotPanel::OnOutputButtonClicked);
        }
        m_OutputGroupBox.setLayout(&m_OutputLayout);
        m_OutputGroupBox.setTitle("Digital outputs:");
        m_MainLayout.addRow(&m_OutputGroupBox);
        m_pJointControls=new JointControl*[m_CountJoints];
        for(uint32_t i=0;i<m_CountJoints;i++)
        {
            m_pJointControls[i]=new JointControl(i,this);
            m_JointsLayout.addWidget(m_pJointControls[i],i/2,i%2);
        }
        m_MainLayout.addRow(&m_JointsLayout);
        setLayout(&m_MainLayout);
        OnStatusFlagsChanged();
        connect(&m_ConnectButton, &QAbstractButton::clicked, this, &RobotPanel::OnConnectButtonClicked);
        connect(&m_EnableButton, &QAbstractButton::clicked, this, &RobotPanel::OnEnableButtonClicked);
        connect(&m_ReferenceButton, &QAbstractButton::clicked, this, &RobotPanel::OnReferenceButtonClicked);
        connect(&m_ZeroButton, &QAbstractButton::clicked, this, &RobotPanel::OnZeroButtonClicked);
        connect(&m_OverrideSlider,&QAbstractSlider::valueChanged,this,&RobotPanel::OnOverrideSliderValueChanged);
    }

    //! \brief Callback slot handling clicks to the digital outputs buttons.
    //! \param tag Channel number associated with the clicked button.
    void RobotPanel::OnOutputButtonClicked(int tag, bool bChecked)
    {
        if(!m_pOutputs[tag]->isChecked())
            RobotCommand(cpr_robot::Robot::COMMAND_DOUT_DISABLE, 0.0,tag);
        else
            RobotCommand(cpr_robot::Robot::COMMAND_DOUT_ENABLE, 0.0,tag);
    }
  

    //! \brief Callback slot handling changes to the override slider value.
    void RobotPanel::OnOverrideSliderValueChanged(int value)
    {
        RobotCommand(cpr_robot::Robot::COMMAND_OVERRIDE, 0.01*(double)m_OverrideSlider.value(),0);
    }

    //! \brief Callback slot handling clicks to the "Connect/Disconnect" button.
    void RobotPanel::OnConnectButtonClicked(bool bChecked)
    {
        if(m_StatusFlags&cpr_robot::Robot::STATUSFLAG_DISCONNECTED)
            RobotCommand(cpr_robot::Robot::COMMAND_CONNECT, 0.0, 0);
        else
            RobotCommand(cpr_robot::Robot::COMMAND_DISCONNECT, 0.0, 0);
    }

    //! \brief Callback slot handling clicks to the "Reference" button.
    void RobotPanel::OnReferenceButtonClicked(bool bChecked)
    {
        RobotCommand(cpr_robot::Robot::COMMAND_STARTREFERENCING, 0.0, 0);
    }

    //! \brief Callback slot handling clicks to the "Set Zero" button.
    void RobotPanel::OnZeroButtonClicked(bool bChecked)
    {
        RobotCommand(cpr_robot::Robot::COMMAND_SETZERO, 0.0, 0);
    }

    //! \brief Callback slot handling clicks to the "Enable/Disable" button.
    void RobotPanel::OnEnableButtonClicked(bool bChecked)
    {
        if(m_StatusFlags&cpr_robot::MotorModule::STATUSFLAG_MASK_ANY_ERROR)
            RobotCommand(cpr_robot::Robot::COMMAND_ENABLE, 0.0, 0);
        else
            RobotCommand(cpr_robot::Robot::COMMAND_DISABLE, 0.0, 0);
    }

    //! \brief Initializes all relevant members.
    void RobotPanel::InitializeState()
    {
        cpr_robot::GetRobotInfoResponse rbtInfo=GetRobotInfo();
        m_CountJoints=rbtInfo.CountJoints;
        m_Inputs.clear();
        for(size_t i=0;i<rbtInfo.InputChannels.size();i++)
            m_Inputs.push_back(rbtInfo.InputChannels[i]);
        m_Outputs.clear();
        for(size_t i=0;i<rbtInfo.OutputChannels.size();i++)
            m_Outputs.push_back(rbtInfo.OutputChannels[i]);
        m_ModelName=rbtInfo.Model;
        m_StatusFlags=cpr_robot::Robot::STATUSFLAG_DISCONNECTED;
    }
    
    //! \brief Sets up communication with ROS.
    void RobotPanel::InitializeROS()
    {
        m_GetRobotInfoClient=m_Node.serviceClient<cpr_robot::GetRobotInfo>("/GetRobotInfo");
        m_RobotStateSubscriber=m_Node.subscribe("/robot_state",10,&RobotPanel::RobotStateCallback, this);
        m_RobotCommandClient= m_Node.serviceClient<cpr_robot::RobotCommand>("/RobotCommand");
        m_InputChannelsSubscriber=m_Node.subscribe("/InputChannels",10,&RobotPanel::InputChannelsCallback, this);
        m_OutputChannelsSubscriber=m_Node.subscribe("/OutputChannels",10,&RobotPanel::OutputChannelsCallback, this);
    }

    //! \brief Callback that handles messages received over the /RobotState ROS topic.
    //! \param msg The received message.
    void RobotPanel::RobotStateCallback(const cpr_robot::RobotState::ConstPtr& msg)
    {
        if(m_StatusFlags!=msg->StatusFlags)
        {
            m_StatusFlags=msg->StatusFlags;
            OnStatusFlagsChanged();
        }
        if(m_Override!=msg->Override)
        {
            m_Override=msg->Override;
            OnOverrideChanged();
        }
        if(!m_OverrideSliderValid)
        {
            if((m_StatusFlags&0xffff)==0x0000)
            {
                m_OverrideSlider.setEnabled(true);
                m_OverrideSlider.setValue((int)(100.0*m_Override));
                m_OverrideSliderValid=true;
            }
        }
        for(uint32_t i=0;i<m_CountJoints;i++)
        {
            if(msg->StatusFlags&(((uint32_t)1)<<(16+i)))
                m_pJointControls[i]->set_IsReferenced(true);
            else
                m_pJointControls[i]->set_IsReferenced(false);
        }
    }

    //! \brief Callback that handles messages received over the /InputChannels ROS topic.
    //! \param msg The received message.
    void RobotPanel::InputChannelsCallback(const cpr_robot::ChannelStates::ConstPtr& msg)
    {
        for(size_t i=0;i<msg->state.size();i++)
        {
            m_pInputs[i]->setCheckState(msg->state[i]?Qt::Checked:Qt::Unchecked);
        }
    }

    //! \brief Callback that handles messages received over the /InputChannels ROS topic.
    //! \param msg The received message.
    void RobotPanel::OutputChannelsCallback(const cpr_robot::ChannelStates::ConstPtr& msg)
    {
        for(size_t i=0;i<msg->state.size();i++)
        {
            m_pOutputs[i]->setChecked(msg->state[i]);
        }
    }

    //! Updates all UI elements associated with the override value of the robot.
    void RobotPanel::OnOverrideChanged()
    {
        std::stringstream sstr;
        sstr << ::std::fixed << std::setprecision(1) << 100.0*m_Override << " %";
        m_OverrideLabel.setText(sstr.str().c_str());
    }

    //! Updates all UI elements associated with the status flag reported by the robot.
    void RobotPanel::OnStatusFlagsChanged()
    {
        if(m_StatusFlags&cpr_robot::Robot::STATUSFLAG_DISCONNECTED)
        {
            m_ConnectButton.setText(tr("Connect"));
            m_EnableButton.setEnabled(false);
            m_OverrideSlider.setEnabled(false);
            m_ReferenceButton.setEnabled(false);
            m_EnableButton.setText(tr("Enable"));
            for(uint32_t i=0;i<m_CountJoints;i++)
                m_pJointControls[i]->Disable();
            for(size_t i=0;i<m_Inputs.size();i++)
                m_pInputs[i]->setEnabled(false);
            for(size_t i=0;i<m_Outputs.size();i++)
                m_pOutputs[i]->setEnabled(false);
        }
        else
        {
            m_ConnectButton.setText(tr("Disconnect"));
            m_EnableButton.setEnabled(true);
            m_OverrideSlider.setEnabled(false);
            if(m_StatusFlags&cpr_robot::MotorModule::STATUSFLAG_MASK_ANY_ERROR)
            {
                for(uint32_t i=0;i<m_CountJoints;i++)
                    m_pJointControls[i]->Disable();
                m_EnableButton.setText(tr("Enable"));
                m_ReferenceButton.setEnabled(false);
                for(size_t i=0;i<m_Inputs.size();i++)
                    m_pInputs[i]->setEnabled(false);
                for(size_t i=0;i<m_Outputs.size();i++)
                    m_pOutputs[i]->setEnabled(false);
            }
            else
            {
                for(uint32_t i=0;i<m_CountJoints;i++)
                    m_pJointControls[i]->Enable();
                m_EnableButton.setText(tr("Disable"));
                m_ReferenceButton.setEnabled(true);
                for(size_t i=0;i<m_Inputs.size();i++)
                    m_pInputs[i]->setEnabled(true);
                for(size_t i=0;i<m_Outputs.size();i++)
                    m_pOutputs[i]->setEnabled(true);
            }
        }
        m_OverrideSliderValid=false;
    }

    //! \brief Queries information about the robot from the /GetRobotInfo ROS service.
    cpr_robot::GetRobotInfoResponse RobotPanel::GetRobotInfo()
    {
        cpr_robot::GetRobotInfo srv;
        srv.request.Sender="cpr_robot::RobotPanel";
        if (m_GetRobotInfoClient.call(srv))
        {
            ROS_INFO("GetRobotInfo: model %s, %ui joints.",srv.response.Model.c_str(),srv.response.CountJoints);
        }
        else
        {
            ROS_ERROR("Failed to call service GetRobotInfo.");
            srv.response.Model="Unknown Robot";
            srv.response.CountJoints=0;
        }
        return srv.response;
    }

    //! \brief Sends a command to the robot using the /RobotCommand ROS service.
    //! \param commandId The ID of the command that will be sent.
    //! \param payloadFloat A floating point number that may be provided as data with the command.
    //! \param payloadInt An integer number that may be provided as data with the command.
    cpr_robot::RobotCommandResponse RobotPanel::RobotCommand(const uint32_t commandId, const double payloadFloat, const int64_t payloadInt)
    {
        cpr_robot::RobotCommand srv;
        srv.request.Sender="cpr_robot::RobotPanel";
        srv.request.CommandId=commandId;
        srv.request.PayloadFloat=payloadFloat;
        srv.request.PayloadInt=payloadInt;
        if (!m_RobotCommandClient.call(srv))
        {
            ROS_ERROR("Failed to call service RobotCommand.");
        }
        return srv.response;
    }


}

// Tell pluginlib about this class. 
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cpr_rviz::RobotPanel,rviz::Panel)

#include "moc_RobotPanel.cpp"
