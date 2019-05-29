#pragma once

namespace cpr_rviz
{
	//! \class JointControl JointControl.h <JointControl.h>
	//! \brief Widget that provides motion control for a single robot joint. 
    class JointControl: public QWidget
    {
    	Q_OBJECT
    private:
        //! A handle to the current ROS node.
        ros::NodeHandle m_Node;
        //! Client used to query information about the joint via the /GetJointInfo ROS service.
        ros::ServiceClient m_GetJointInfoClient;
        //! Publisher used to publish jog commands over the /JointJog ROS topic.
        ros::Publisher m_JointJogPublisher;
        //! Subscriber that will listen for joint states on the /joint_states ROS topic.
        ros::Subscriber m_JointStateSubscriber;
        //! The ID of the joint.
        uint32_t m_JointId;
        //! The name of the joint as used in ROS topics and services.
        std::string m_JointName;
        //! The position of the joint in radians.
        double m_Position;
        //! The angular velocity of the joint in radians per second.
        double m_Velocity;
        //! Flag indicating whether the joint has been referenced.
        bool m_bIsReferenced;
        //! The joint type. Currently this is not used
        uint32_t m_JointType;
        //! The top-level layout of the widget.
        QHBoxLayout m_MainLayout;
        //! A group box widget that will contain the controls.
        QGroupBox m_GroupBox;
        //! The layout arranging the controls in the group box.
        QFormLayout m_GroupLayout;
        //! A label that will show the current position.
        QLabel m_PositionLabel;
        //! A label that will show the current velocity.
        QLabel m_VelocityLabel;
        //! A slider that allows to control the desired velocity.
        QSlider m_VelocitySlider;
        //! A button that will set the desired velocity to zero when pressed.
        QPushButton m_StopButton;

        void OnVelocityChanged();
        void OnPositionChanged();

        void InitializeUI();
        void InitializeROS();
        void InitializeState();
        void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    protected slots:
        void OnVelocitySliderValueChanged(int value);
        void OnStopButtonClicked(bool checked = false);
    protected:
        cpr_robot::GetJointInfoResponse GetJointInfo();
    public:
        void set_IsReferenced(const bool bReferenced);
        bool get_Referenced() const;
        void Enable();
        void Disable();
 	    JointControl(const uint32_t jointId, QWidget* parent = nullptr );
    };
}