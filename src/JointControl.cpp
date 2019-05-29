#include <cpr_robot.h>
#include "qt_includes.h"
#include "JointControl.h"
#include <sstream>
#include <iomanip>

namespace cpr_rviz
{
    //! \brief Constructs a JointControl widget.
    //! \param jointId The ID of the joint that will be controlled by the widget.
    //! \param parent The parent widget
    JointControl::JointControl(const uint32_t jointId,QWidget* parent ) :
        m_GroupBox(this),
        m_JointId(jointId),
        m_PositionLabel(&m_GroupBox),
        m_VelocityLabel(&m_GroupBox),
        m_VelocitySlider(Qt::Horizontal,&m_GroupBox),
        m_StopButton(&m_GroupBox),
        m_bIsReferenced(false)
    {
        InitializeROS();
        InitializeState();
        InitializeUI();
    }
    
    //! \brief Initializes all UI elements.
    void JointControl::InitializeUI()
    {
        m_GroupLayout.addRow(tr("Position:"),&m_PositionLabel);
        m_GroupLayout.addRow(tr("Velocity:"),&m_VelocityLabel);
        m_GroupLayout.addRow(&m_VelocitySlider);
        m_GroupLayout.addRow(&m_StopButton);
        m_StopButton.setText(tr("STOP"));
        m_VelocitySlider.setMinimum(-100);
        m_VelocitySlider.setMaximum(100);
        m_VelocitySlider.setTickInterval(100);
        m_VelocitySlider.setTickPosition(QSlider::TicksBelow);
        m_GroupBox.setLayout(&m_GroupLayout);
        m_GroupBox.setTitle((m_JointName+":").c_str());
        m_MainLayout.addWidget(&m_GroupBox);
        setLayout(&m_MainLayout);
        OnPositionChanged();
        OnVelocityChanged();
        Disable();
        connect(&m_VelocitySlider,&QAbstractSlider::valueChanged,this,&JointControl::OnVelocitySliderValueChanged);
        connect(&m_StopButton,&QAbstractButton::clicked,this,&JointControl::OnStopButtonClicked);
    }
    
    //! \brief Sets the referencing state.
    //! \param bReferenced Flag indicating whether the joint has been referenced. 
    void JointControl::set_IsReferenced(const bool bReferenced)
    {
        if(m_bIsReferenced!=bReferenced)
        {
            m_bIsReferenced=bReferenced;
            OnPositionChanged();
        }
    }
        
    //! \brief Gets the referencing state.
    //! \return Flag indicating whether the joint has been referenced. 
    bool JointControl::get_Referenced() const
    {
        return m_bIsReferenced;
    }

    //! \brief Enables all relevant UI elements.
    void JointControl::Enable()
    {
        m_VelocitySlider.setValue(0);
        m_VelocitySlider.setEnabled(true);
        m_StopButton.setEnabled(true);
    }

    //! \brief Disables all relevant UI elements.
    void JointControl::Disable()
    {
        m_VelocitySlider.setValue(0);
        m_VelocitySlider.setEnabled(false);
        m_StopButton.setEnabled(false);
    }

    //! \brief Callback slot that is called when the STOP button is clicked.
    void JointControl::OnStopButtonClicked(bool checked)
    {
        m_VelocitySlider.setValue(0);
    }

    //! \brief Updates the UI elements associated with velocity.
    void JointControl::OnVelocityChanged()
    {
        std::stringstream sstr;
        sstr << ::std::fixed << std::setprecision(1) << (180.0*m_Velocity/M_PI) << " °/s";
        m_VelocityLabel.setText(sstr.str().c_str());
    }

    //! \brief Updates the UI elements associated with position.
    void JointControl::OnPositionChanged()
    {
        std::stringstream sstr;
        sstr << ::std::fixed << std::setprecision(1) << (180.0*m_Position/M_PI) << " °";
        if(!m_bIsReferenced)
            sstr << " (unreferenced)";
        m_PositionLabel.setText(sstr.str().c_str());
    }

    //! \brief Initializes all relevant members.
    void JointControl::InitializeState()
    {
        cpr_robot::GetJointInfoResponse jointInfo=GetJointInfo();
        m_JointName=jointInfo.JointName;
        m_JointType=jointInfo.JointType;
        m_Position=0.0;
        m_Velocity=0.0;
    }
    
    //! \brief Sets up communication with ROS.
    void JointControl::InitializeROS()
    {
        m_GetJointInfoClient=m_Node.serviceClient<cpr_robot::GetJointInfo>("/GetJointInfo");
        m_JointJogPublisher=m_Node.advertise<control_msgs::JointJog>("/JointJog", 50);
        m_JointStateSubscriber=m_Node.subscribe("/joint_states",10,&JointControl::JointStateCallback, this);
    }

    //! \brief Callback that handles messages received over the /JointState ROS topic.
    //! \param msg The received message.
    void JointControl::JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        for(size_t i=0;i<msg->name.size();i++)
        {
            if(msg->name[i]==m_JointName)
            {
                double velocity=msg->velocity[i];
                if(velocity!=m_Velocity)
                {
                    m_Velocity=velocity;
                    OnVelocityChanged();
                }
                double position=msg->position[i];
                if(position!=m_Position)
                {
                    m_Position=position;
                    OnPositionChanged();
                }
            }
        }
    }

    //! \brief Callback slot that handles changes to the velocity control slider.
    void JointControl::OnVelocitySliderValueChanged(int value)
    {
        control_msgs::JointJog msg;
        msg.header.stamp=ros::Time::now();
        msg.joint_names.push_back(m_JointName);
        double velocity=0.01*(double)value;
        msg.velocities.push_back(velocity);
        m_JointJogPublisher.publish(msg);
    }

    //! \brief Queries information about the joint from the /GetJointInfo ROS service.
    cpr_robot::GetJointInfoResponse JointControl::GetJointInfo()
    {
        cpr_robot::GetJointInfo srv;
        srv.request.Sender="cpr_robot::JointControl";
        srv.request.JointId=m_JointId;
        if (m_GetJointInfoClient.call(srv))
        {
            ROS_INFO("GetJointInfo: %s, type %ui.",srv.response.JointName.c_str(),srv.response.JointType);
        }
        else
        {
            ROS_ERROR("Failed to call service GetJointInfo.");
            srv.response.JointName="unknown_joint";
            srv.response.JointType=0;
        }
        return srv.response;
    }
}

#include "moc_JointControl.cpp"