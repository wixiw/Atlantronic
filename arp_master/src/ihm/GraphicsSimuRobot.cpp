#include "GraphicsSimuRobot.hpp"

#include <wx/wx.h>
#include <ros/package.h>

using namespace arp_core;

using namespace arp_master;

GraphicsSimuRobot::GraphicsSimuRobot(const ros::NodeHandle& nh,
        const wxImage& robot_image, const Vector2& pos, double orient,
        double one_meter_in_pixel, std::string topicName) :
    nh_(nh), m_realPosition(pos), m_realHeading(orient),
    m_computedPosition(pos), m_computedHeading(orient),
    old_pos_(pos), old_orient_(orient),
            robot_image_(robot_image), pen_on_(true),
            pen_(wxColour(DEFAULT_PEN_R, DEFAULT_PEN_G, DEFAULT_PEN_B)),
            m_realCanvasX(0.0), m_realCanvasY(0.0), old_canvas_x_(0.0),
            old_canvas_y_(0.0), meter_(one_meter_in_pixel)
{
    pen_.SetWidth(3);
    robot_ = wxBitmap(robot_image_);

    std::string images_path = ros::package::getPath("arp_master")
             + "/ressource/images/robot_light.png"  ;
    m_robotLightImage.LoadFile(
            wxString::FromAscii(images_path.c_str()));
    m_robotLightImage.SetMask(true);
    m_robotLightImage.SetMaskColour(255, 255, 255);
    m_robotLight = wxBitmap(m_robotLightImage);

    // Suscribers
    m_realPosSub = ros::NodeHandle().subscribe(topicName, 1,
            &GraphicsSimuRobot::realPoseCallback, this);

    // Suscribers
    m_computedPosSub = ros::NodeHandle().subscribe("Localizator/pose", 1,
            &GraphicsSimuRobot::computedPoseCallback, this);

    // Services
    set_pen_srv_ = nh_.advertiseService("set_pen",
            &GraphicsSimuRobot::setPenCallback, this);

}

void GraphicsSimuRobot::realPoseCallback(const arp_core::PoseConstPtr& p)
{
    m_realPosition = Vector2(p->x, p->y);
    m_realHeading = Rotation2(p->theta);
}

void GraphicsSimuRobot::computedPoseCallback(const arp_core::PoseConstPtr& p)
{
    m_computedPosition = Vector2(p->x, p->y);
    m_computedHeading = Rotation2(p->theta);
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

    m_realPosition.x() = std::min(std::max(m_realPosition.x(), -canvas_width / (double) 2.0),
            canvas_width / (double) 2.0);
    m_realPosition.y() = std::min(std::max(m_realPosition.y(), -canvas_height / (double) 2.0),
            canvas_height / (double) 2.0);

    // Conversion en coordonnees image
    m_realCanvasX = (canvas_width / 2.0 + m_realPosition.x()) * meter_;
    m_realCanvasY = (canvas_height / 2.0 - m_realPosition.y()) * meter_;
    m_computedCanvasX = (canvas_width / 2.0 + m_computedPosition.x()) * meter_;
    m_computedCanvasY = (canvas_height / 2.0 - m_computedPosition.y()) * meter_;

    {
        wxImage rotated_image = robot_image_.Rotate(
                m_realHeading.angle() - PI / 2.0,
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


    {
        wxImage rotated_image = m_robotLightImage.Rotate(
                m_computedHeading.angle() - PI / 2.0,
                wxPoint(m_robotLightImage.GetWidth() / 2,
                        m_robotLightImage.GetHeight() / 2), false);

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

        m_robotLight = wxBitmap(rotated_image);
    }

    ROS_DEBUG("[%s]: pos_x: %f pos_y: %f theta: %f",
            nh_.getNamespace().c_str(), m_realPosition.x(), m_realPosition.y(), m_realHeading.angle());

    if (pen_on_)
    {
        if (m_realPosition != old_pos_)
        {
            path_dc.SetPen(pen_);
            path_dc.DrawLine(m_realCanvasX, m_realCanvasY, old_canvas_x_, old_canvas_y_);
        }
    }

    // Buffer
    old_pos_ = m_realPosition;
    old_orient_ = m_realHeading;
    old_canvas_x_ = m_realCanvasX;
    old_canvas_y_ = m_realCanvasY;
}

void GraphicsSimuRobot::paint(wxDC& dc)
{
    dc.DrawBitmap(m_robotLight, m_computedCanvasX - (m_robotLight.GetWidth() / 2),
            m_computedCanvasY - (m_robotLight.GetHeight() / 2), true);

    dc.DrawBitmap(robot_, m_realCanvasX - (robot_.GetWidth() / 2),
            m_realCanvasY - (robot_.GetHeight() / 2), true);

}

