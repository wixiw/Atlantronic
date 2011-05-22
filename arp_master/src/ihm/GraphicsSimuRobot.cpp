#include "GraphicsSimuRobot.hpp"

#include <wx/wx.h>

using namespace arp_core;

using namespace arp_master;

GraphicsSimuRobot::GraphicsSimuRobot(const ros::NodeHandle& nh,
        const wxImage& robot_image, const Vector2& pos, double orient,
        double one_meter_in_pixel, std::string topicName) :
    nh_(nh), pos_(pos), orient_(orient), old_pos_(pos), old_orient_(orient),
            robot_image_(robot_image), pen_on_(true),
            pen_(wxColour(DEFAULT_PEN_R, DEFAULT_PEN_G, DEFAULT_PEN_B)),
            canvas_x_(0.0), canvas_y_(0.0), old_canvas_x_(0.0),
            old_canvas_y_(0.0), meter_(one_meter_in_pixel)
{
    pen_.SetWidth(3);
    robot_ = wxBitmap(robot_image_);

    // Suscribers
    pose_sub_ = ros::NodeHandle().subscribe(topicName, 1,
            &GraphicsSimuRobot::poseCallback, this);

    // Services
    set_pen_srv_ = nh_.advertiseService("set_pen",
            &GraphicsSimuRobot::setPenCallback, this);

}

void GraphicsSimuRobot::poseCallback(const arp_core::PoseConstPtr& p)
{
    pos_ = Vector2(p->x, p->y);
    orient_ = Rotation2(p->theta);
}

bool GraphicsSimuRobot::setPenCallback(SetPen::Request& req, SetPen::Response&)
{
    pen_on_ = !req.off;
    if (req.off)
    {
        return true;
    }

    wxPen pen(wxColour(req.r, req.g, req.b));
    if (req.width != 0)
    {
        pen.SetWidth(req.width);
    }

    pen_ = pen;
    return true;
}

void GraphicsSimuRobot::update(wxMemoryDC& path_dc, float canvas_width,
        float canvas_height)
{

    pos_.x() = std::min(std::max(pos_.x(), -canvas_width / (double) 2.0),
            canvas_width / (double) 2.0);
    pos_.y() = std::min(std::max(pos_.y(), -canvas_height / (double) 2.0),
            canvas_height / (double) 2.0);

    // Conversion en coordonnees image
    canvas_x_ = (canvas_width / 2.0 + pos_.x()) * meter_;
    canvas_y_ = (canvas_height / 2.0 - pos_.y()) * meter_;

    {
        wxImage rotated_image = robot_image_.Rotate(
                orient_.angle() - PI / 2.0,
                wxPoint(robot_image_.GetWidth() / 2,
                        robot_image_.GetHeight() / 2), false);

        for (int y = 0; y < rotated_image.GetHeight(); ++y)
        {
            for (int x = 0; x < rotated_image.GetWidth(); ++x)
            {
                if (rotated_image.GetRed(x, y) == 255 && rotated_image.GetBlue(
                        x, y) == 255 && rotated_image.GetGreen(x, y) == 255)
                {
                    rotated_image.SetAlpha(x, y, 0);
                }
            }
        }

        robot_ = wxBitmap(rotated_image);
    }

    ROS_DEBUG("[%s]: pos_x: %f pos_y: %f theta: %f",
            nh_.getNamespace().c_str(), pos_.x(), pos_.y(), orient_.angle());

    if (pen_on_)
    {
        if (pos_ != old_pos_)
        {
            path_dc.SetPen(pen_);
            path_dc.DrawLine(canvas_x_, canvas_y_, old_canvas_x_, old_canvas_y_);
        }
    }

    // Buffer
    old_pos_ = pos_;
    old_orient_ = orient_;
    old_canvas_x_ = canvas_x_;
    old_canvas_y_ = canvas_y_;
}

void GraphicsSimuRobot::paint(wxDC& dc)
{
    dc.DrawBitmap(robot_, canvas_x_ - (robot_.GetWidth() / 2),
            canvas_y_ - (robot_.GetHeight() / 2), true);

}

