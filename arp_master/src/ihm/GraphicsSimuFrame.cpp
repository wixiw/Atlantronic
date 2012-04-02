/*
 * GraphicsSimuFrame.cpp
 *
 *  Created on: april 2011
 *      Author: bmo
 */

#include "GraphicsSimuFrame.hpp"

#include <ros/package.h>
#include <cstdlib>
#include <ctime>

using namespace arp_master;

GraphicsSimuFrame::GraphicsSimuFrame(std::string topicName) :
            wxFrame(NULL, wxID_ANY, wxT("GraphicsSimu"), wxDefaultPosition,
                    wxSize(table_length_in_pixel, table_width_in_pixel),
                    wxDEFAULT_FRAME_STYLE),
            nh_(ros::NodeHandle("GraphicsSimu"))
{

    update_timer_ = new wxTimer(this);
    update_timer_->Start(100.);

    Connect(update_timer_->GetId(), wxEVT_TIMER,
            wxTimerEventHandler(GraphicsSimuFrame::onUpdate), NULL, this);
    Connect(wxEVT_PAINT, wxPaintEventHandler(GraphicsSimuFrame::onPaint), NULL,
            this);

    std::string images_path = ros::package::getPath("arp_master")
            + "/ressource/images/";

    std::string robot_image_file = "box-turtle.png"; // hauteur de robot.png : 200px = 1m
    robot_image_.LoadFile(
            wxString::FromAscii((images_path + robot_image_file).c_str()));
    robot_image_.SetMask(true);
    robot_image_.SetMaskColour(255, 255, 255);

    std::string table_image_file = "table.png"; // 609 x 429 px
    table_image_.LoadFile(
            wxString::FromAscii((images_path + table_image_file).c_str()));
    table_image_.SetMask(false);

    table_bitmap_ = wxBitmap(table_image_);
    ROS_INFO("table image : %d x %d", table_image_.GetHeight(),
            table_image_.GetWidth());
    path_dc_.SelectObject(table_bitmap_);
    clear();

    clear_srv_ = nh_.advertiseService("clear",
            &GraphicsSimuFrame::clearCallback, this);

    ROS_INFO("Starting GraphicsSimulator with node name %s",
            ros::this_node::getName().c_str());

    GraphicsSimuRobotPtr t(
            new GraphicsSimuRobot(ros::NodeHandle(), robot_image_,
                    Vector2(0., 0.), 0., one_meter_in_pixel, topicName));
    mRobot = t;

    /*
    wxStaticText* position = new wxStaticText(this, wxID_ANY, wxT("Measured :"),wxPoint(10, 90));
    wxString  position_x  = wxT("X : -1000 mm");
    wxStaticText* position_x_text = new wxStaticText(this, wxID_ANY, position_x, wxPoint(10, 105));
    wxStaticText* position_y_text = new wxStaticText(this, wxID_ANY, wxT("Y : 1000 mm"), wxPoint(10, 120));
    wxStaticText* position_theta_text = new wxStaticText(this, wxID_ANY, wxT("theta : 135°"), wxPoint(10, 135));
    position_x = wxT("X : 0 mm");*/

}

GraphicsSimuFrame::~GraphicsSimuFrame()
{
    delete update_timer_;
}

void GraphicsSimuFrame::clear()
{
    path_dc_.SetBackground(wxBrush(table_image_));
    path_dc_.Clear();
}

void GraphicsSimuFrame::onUpdate(wxTimerEvent& evt)
{
    ros::spinOnce();

    mRobot->update(path_dc_, table_length_in_pixel / one_meter_in_pixel,
            table_width_in_pixel / one_meter_in_pixel);

    Refresh();

    if (!ros::ok())
    {
        Close();
    }
}

void GraphicsSimuFrame::onPaint(wxPaintEvent& evt)
{
    wxPaintDC dc(this);

    dc.DrawBitmap(table_bitmap_, 0, 0, true);

    mRobot->paint(dc);

}

bool GraphicsSimuFrame::clearCallback(std_srvs::Empty::Request&,
        std_srvs::Empty::Response&)
{
    ROS_INFO("Clearing GraphicsSimulator.");
    clear();
    return true;
}

