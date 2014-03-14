/*
 * RosLinkedButton.hpp
 *
 *  Created on: Mar 13, 2014
 *      Author: willy
 */

#ifndef ROSLINKEDBUTTON_HPP_
#define ROSLINKEDBUTTON_HPP_

#define wxUSE_GUI 1
#include <wx/wx.h>
#include <wx/wxchar.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace arp_hml
{

/**
 * Bouton représentant le capteur Omron visant sous la ventouse gauche à la verticale
 * Used to publish on "Ubiquity2014/leftArmSuckerTor"
 */

class RosLinkedButton
{
     public:

        RosLinkedButton();
        ~RosLinkedButton();

        void init(int id, wxPoint position, const char* name, const char* topicName,wxFrame* parentFrame);
        void onButonEvent(wxCommandEvent& event);
        void publish();

     protected:
        int             m_id;
        std_msgs::Bool  m_value;
        wxButton*       m_widget;
        ros::Publisher  m_rosPublisher;

};

} /* namespace arp_hml2 */
#endif /* ROSLINKEDBUTTON_HPP_ */
