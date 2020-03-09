#include "qt_includes.h"
#include "TaggedButton.h"

namespace cpr_rviz
{
    int TaggedButton::get_Tag() const
    {
        return m_Tag;
    }

    TaggedButton::TaggedButton(const int tag, QWidget* pParent) : 
        QPushButton(pParent),
        m_Tag(tag)
    {
        connect(this,&QAbstractButton::clicked,this,&TaggedButton::OnClicked);
    }

    TaggedButton::~TaggedButton()
    {
    }

    void TaggedButton::OnClicked(bool bChecked)
    {
        emit buttonClicked(m_Tag,bChecked);
    }
}


#include "moc_TaggedButton.cpp"