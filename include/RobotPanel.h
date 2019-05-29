#pragma once

namespace cpr_rviz
{
	//! \class RobotPanel RobotPanel.h <RobotPanel.h>
	//! \brief Plugin for RViz that allows to move a robot remotely over ROS.
    class RobotPanel: public rviz::Panel
    {
    	Q_OBJECT
    private:
        //! A handle to the current ROS node.
        ros::NodeHandle m_Node;
        //! Client used to query information about the robot via the /GetRobotInfo ROS service.
        ros::ServiceClient m_GetRobotInfoClient;
        //! Client used to send commands to the robot via the /RobotCommand ROS service.
        ros::ServiceClient m_RobotCommandClient;
        //! Subscriber listening to the /RobotState ROS topic.
        ros::Subscriber m_RobotStateSubscriber;
        //! The model designation reported by the robot.
        std::string m_ModelName;
        //! The number of joints reported by the robot.
        uint32_t m_CountJoints;
        //! Layout aranging the control buttons horizontally.
        QHBoxLayout m_ControlButtonsLayout;
        //! Button allowing to connect/disconnect the robot remotely.
        QPushButton m_ConnectButton;
        //! Button allowing to enable/disable motor motion on the robot remotely.
        QPushButton m_EnableButton;
        //! Button allowing to start the referencing procedure on the robot remotely.
        QPushButton m_ReferenceButton;
        //! Button allowing to set the encoder positions on the robot remotely to zero.
        QPushButton m_ZeroButton;
        //! Label used to show the current override value of the robot.
        QLabel m_OverrideLabel;
        //! Slider used to change the desired override value of the robot.
        QSlider m_OverrideSlider;
        //! Layout arranging the controls of the individual joints.
        QGridLayout m_JointsLayout;
        //! The top-level layout of this widget.
        QFormLayout m_MainLayout;
        //! Label used to show the model deisgnation of the robot.
        QLabel m_ModelNameLabel;
        //! The status flags reported by the robot.
        uint32_t m_StatusFlags;
        //! The override value reported by the robot.
        double m_Override;
        //! Flag indicating whether the position of the override slider has been properly initialized.
        bool m_OverrideSliderValid;
        //! Array containing the control widgets for the individual joints.
        JointControl** m_pJointControls;

        void InitializeUI();
        void InitializeROS();
        void InitializeState();

        void RobotStateCallback(const cpr_robot::RobotState::ConstPtr& msg);

        void OnStatusFlagsChanged();
        void OnOverrideChanged();
    protected:
        cpr_robot::GetRobotInfoResponse GetRobotInfo();
        cpr_robot::RobotCommandResponse RobotCommand(const uint32_t commandId, const double payloadFloat, const int64_t payloadInt);
    protected slots:
        void OnOverrideSliderValueChanged(int value);
        void OnConnectButtonClicked(bool bChecked = true);
        void OnEnableButtonClicked(bool bChecked = true);
        void OnReferenceButtonClicked(bool bChecked = true);
        void OnZeroButtonClicked(bool bChecked = true);
    public:
	    RobotPanel( QWidget* parent = nullptr );
        virtual ~RobotPanel();
    };

}

//! \namespace cpr_rviz Provides classes defining a plugin for RViz.
