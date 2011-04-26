#include "SimuFrame.hpp"

#include <ros/package.h>
#include <cstdlib>
#include <ctime>


using namespace arp_master;

SimuFrame::SimuFrame()
: wxFrame(NULL,
          wxID_ANY,
          wxT("ARDSimulator"), 
          wxDefaultPosition, 
          wxSize(table_length_in_pixel, table_width_in_pixel),
          wxDEFAULT_FRAME_STYLE)
, nh_(ros::NodeHandle("ARDSimu"))
{
  srand(time(NULL));

  update_timer_ = new wxTimer(this);
  update_timer_->Start(default_dt_ms);

  Connect(update_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler(SimuFrame::onUpdate), NULL, this);
  Connect(wxEVT_PAINT, wxPaintEventHandler(SimuFrame::onPaint), NULL, this);


  std::string images_path = ros::package::getPath("arp_master") + "/ressource/images/";

  std::string robot_image_file = "robot.png";  // hauteur de robot.png : 200px = 1m
  robot_image_.LoadFile(wxString::FromAscii((images_path + robot_image_file).c_str()));
  robot_image_.SetMask(true);
  robot_image_.SetMaskColour(255, 255, 255);

  std::string table_image_file = "table.png";  // 609 x 429 px
  table_image_.LoadFile(wxString::FromAscii((images_path + table_image_file).c_str()));
  table_image_.SetMask(true);
  table_image_.SetMask(false);

  table_bitmap_ = wxBitmap(table_image_);
  ROS_INFO("table image : %d x %d", table_image_.GetHeight(), table_image_.GetWidth()) ;
  path_dc_.SelectObject(table_bitmap_);
  clear();

  clear_srv_ = nh_.advertiseService("clear", &SimuFrame::clearCallback, this);
  respawn_srv_ = nh_.advertiseService("respawn", &SimuFrame::respawnCallback, this);

  ROS_INFO("Starting ARDSimu with node name %s", ros::this_node::getName().c_str()) ;

  spawnRobot(0.0, 0.0, 0.0);
}

SimuFrame::~SimuFrame()
{
  delete update_timer_;
}

bool SimuFrame::respawnCallback(Spawn::Request& req, Spawn::Response& res)
{
  ROS_INFO("Respawing ARDSimu to x=%f, y=%f and theta=%f", req.x, req.y, req.theta);
  mRobot.reset();  //"deleting" shared_ptr
  spawnRobot(req.x, req.y, req.theta);
  clear();
  return true;
}


void SimuFrame::spawnRobot(double x, double y, double angle)
{
  SimuRobotPtr t(new SimuRobot(ros::NodeHandle("Protokrot"), robot_image_, Vector2(x, y), angle, one_meter_in_pixel));
  mRobot = t;

  ROS_INFO("Spawning robot [%s] at x=[%f], y=[%f], theta=[%f]", "Protokrot", x, y, angle);

  return;
}

void SimuFrame::clear()
{
  path_dc_.SetBackground(wxBrush( table_image_ ));
  path_dc_.Clear();
}

void SimuFrame::onUpdate(wxTimerEvent& evt)
{
  ros::spinOnce();

  updateRobot();

  if (!ros::ok())
  {
    Close();
  }
}

void SimuFrame::onPaint(wxPaintEvent& evt)
{
  wxPaintDC dc(this);

  dc.DrawBitmap(table_bitmap_, 0, 0, true);

  mRobot->paint(dc);
}

void SimuFrame::updateRobot()
{
  if (last_robot_update_.isZero())
  {
    last_robot_update_ = ros::WallTime::now();
    return;
  }

  Refresh();

  ros::WallTime t = ros::WallTime::now();
  double dt = (t - last_robot_update_).toSec();
  last_robot_update_ = t;
  mRobot->update(dt, 
                 path_dc_, 
                 table_length_in_pixel/one_meter_in_pixel, 
                 table_width_in_pixel/one_meter_in_pixel );

}


bool SimuFrame::clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Clearing ARDSimu.");
  clear();
  return true;
}

