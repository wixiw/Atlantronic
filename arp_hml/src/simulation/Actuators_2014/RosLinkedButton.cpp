/*
 * RosLinkedButton.cpp
 *
 *  Created on: Mar 13, 2014
 *      Author: willy
 */

#include "RosLinkedButton.hpp"

using namespace arp_hml;
using namespace std_msgs;

RosLinkedButton::RosLinkedButton():
        m_id(1),
        m_widget(NULL)
{
    m_value.data = false;
}

RosLinkedButton::~RosLinkedButton()
{
    delete m_widget;
}


void RosLinkedButton::init(int id, wxPoint position, const char*  name, const char* topicName, wxFrame* parentFrame)
{
    m_id = id;

    m_widget = new wxButton(parentFrame,
            m_id,
            wxT("default"),
            position,
            wxDefaultSize);

    ros::NodeHandle nodeHandle;
    m_rosPublisher = nodeHandle.advertise<Bool>(topicName, 1);
}

void RosLinkedButton::onButonEvent(wxCommandEvent& event)
{
    m_value.data = !m_value.data;
}

void RosLinkedButton::publish()
{
    m_rosPublisher.publish(m_value);
}
