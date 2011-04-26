#include "SimuRobot.hpp"

#include <wx/wx.h>

using namespace arp_core;

using namespace arp_master;


SimuRobot::SimuRobot(const ros::NodeHandle& nh, const wxImage& robot_image, const Vector2& pos, double orient, double one_meter_in_pixel)
: nh_(nh)
, robot_image_(robot_image)
, pos_(pos)
, orient_(orient)
, old_pos_(pos)
, old_orient_(orient)
, v_left_(0.0)
, v_right_(0.0)
, lin_vel_(0.0)
, ang_vel_(0.0)
, meter_(one_meter_in_pixel)
, pen_on_(true)
, pen_(wxColour(DEFAULT_PEN_R, DEFAULT_PEN_G, DEFAULT_PEN_B))
, odo_left_(0.0)
, odo_right_(0.0)
, canvas_x_(0.0)
, canvas_y_(0.0)
, old_canvas_x_(0.0)
, old_canvas_y_(0.0)
{
  pen_.SetWidth(3);
  robot_ = wxBitmap(robot_image_);

  // Suscribers
  differential_command_sub_ = nh_.subscribe("differential_command", 1, &SimuRobot::commandCallback, this);

  // Publishers
  pose_pub_ = nh_.advertise<arp_core::Pose>("pose", 1);
  odo_pub_ = nh_.advertise<arp_core::Odo>("odo", 1);

  // Services
  set_pen_srv_ = nh_.advertiseService("set_pen", &SimuRobot::setPenCallback, this);

  // Parameters
  base_line      = 0.4;
  wheel_diameter = 0.07;
  nh_.setParam("/Protokrot/base_line", base_line);
  nh_.setParam("/Protokrot/wheel_diameter", wheel_diameter);

}


void SimuRobot::commandCallback(const arp_core::DifferentialCommandConstPtr& c)
{
  last_command_time_ = ros::WallTime::now();
  v_left_ = c->v_left;
  v_right_ = c->v_right;
}

bool SimuRobot::setPenCallback(SetPen::Request& req, SetPen::Response&)
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


void SimuRobot::update(double dt, wxMemoryDC& path_dc, float canvas_width, float canvas_height)
{
  // Maintient de la commande pendant 0.2 seconde
  if (ros::WallTime::now() - last_command_time_ > ros::WallDuration(0.2))
  {
    v_right_ = 0.0f;
    v_left_ = 0.0f;
  }


  // Calcul du Twist
  lin_vel_ = 0.25 * ( v_right_ + v_left_ ) * wheel_diameter;
  ang_vel_ = 0.50 * ( v_right_ - v_left_ ) * wheel_diameter / base_line;


  // Calcul de la position réelle (integration)
  orient_ = orient_ * Rotation2(ang_vel_ * dt);
  Vector2 delta_trans = lin_vel_ * dt * Vector2(1.0, 0.0);
  pos_ += orient_ * delta_trans;

  
  // les vitesses odo valent la valeur des consignes
  double lin_vel_odo = lin_vel_;
  double ang_vel_odo = ang_vel_;

  // On simule des odo (ils sont suposés parfaits)
  double v_left_odo  = (2.0 * lin_vel_odo + base_line * ang_vel_odo ) / wheel_diameter;
  double v_right_odo = (2.0 * lin_vel_odo - base_line * ang_vel_odo ) / wheel_diameter;
  odo_right_ += v_left_odo  * dt;
  odo_left_  += v_right_odo * dt;
  

  // Clamp to screen size
  if (pos_.x() < -canvas_width/2.0 || pos_.x() >= canvas_width/2.0
      || pos_.y() < -canvas_height/2.0 || pos_.y() >= canvas_height/2.0)
  {
    ROS_WARN("Oh no! I hit the wall! (Clamping from [x=%f, y=%f])", pos_.x(), pos_.y());
  }

  pos_.x() = std::min(std::max(pos_.x(), -canvas_width/(double)2.0), canvas_width/(double)2.0);
  pos_.y() = std::min(std::max(pos_.y(), -canvas_height/(double)2.0), canvas_height/(double)2.0);

  // Conversion en coordonnees image
  canvas_x_ = (canvas_width  / 2.0 + pos_.x()) * meter_;
  canvas_y_ = (canvas_height / 2.0 - pos_.y()) * meter_;

  {
    wxImage rotated_image = robot_image_.Rotate(orient_.angle() - PI/2.0, wxPoint(robot_image_.GetWidth() / 2, robot_image_.GetHeight() / 2), false);

    for (int y = 0; y < rotated_image.GetHeight(); ++y)
    {
      for (int x = 0; x < rotated_image.GetWidth(); ++x)
      {
        if (rotated_image.GetRed(x, y) == 255 && rotated_image.GetBlue(x, y) == 255 && rotated_image.GetGreen(x, y) == 255)
        {
          rotated_image.SetAlpha(x, y, 0);
        }
      }
    }

    robot_ = wxBitmap(rotated_image);
  }


  // Publishing
  Pose p;
  p.x = pos_.x();
  p.y = pos_.y();
  p.theta = fmod(orient_.angle(), 2*PI);
  p.linear_velocity = lin_vel_;
  p.angular_velocity = ang_vel_;
  pose_pub_.publish(p);

  Odo o;
  o.odo_left = odo_left_;
  o.odo_right = odo_right_;
  odo_pub_.publish(o);


  ROS_DEBUG("[%s]: pos_x: %f pos_y: %f theta: %f", nh_.getNamespace().c_str(), pos_.x(), pos_.y(), orient_.angle());

  if (pen_on_)
  {
    if (pos_ != old_pos_)
    {
      path_dc.SetPen(pen_);
      path_dc.DrawLine( canvas_x_,
                        canvas_y_,
                        old_canvas_x_,
                        old_canvas_y_);
    }
  }

  // Buffer
  old_pos_ = pos_;
  old_orient_ = orient_;
  old_canvas_x_ = canvas_x_;
  old_canvas_y_ = canvas_y_;
}

void SimuRobot::paint(wxDC& dc)
{
  dc.DrawBitmap(robot_, 
                canvas_x_ - (robot_.GetWidth() / 2), 
                canvas_y_ - (robot_.GetHeight() / 2),
                true);

}


