#pragma once

namespace cpr_rviz
{
	//! \class RobotPanel RobotPanel.h <RobotPanel.h>
	//! \brief Plugin for RViz that allows to move a robot remotely over ROS.
    class TaggedButton: public QPushButton
    {
    	Q_OBJECT
    private:
        const int m_Tag;
    public:
        int get_Tag() const;
        TaggedButton(const int tag, QWidget* pParent=nullptr);
        virtual ~TaggedButton();
    signals:
        void buttonClicked(int, bool=false);
    private:
        void OnClicked(bool bChecked=false);
    };
}