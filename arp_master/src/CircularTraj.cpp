#include "CircularTraj.hpp"

using namespace arp_master;

CircularTraj::CircularTraj():
  direction_(FORWARD),
  color_(COLOR_RED),
  isRunning(false)
{
}

CircularTraj::~CircularTraj()
{
  pts_.clear();
}

void CircularTraj::push_back(WayPoint & wp)
{ 
  pts_.push_back(wp);
}

void CircularTraj::clear()
{ 
  pts_.clear(); 
  isRunning = false;
}

unsigned int CircularTraj::getNumberOfWayPoints() const 
{ 
  return pts_.size();
}

std::vector<WayPoint> CircularTraj::getWayPointsVector() const
{
  return pts_;
}

void CircularTraj::setIndexPreviousWayPoint(unsigned int index)
{
  setIndexPreviousWayPoint(index, current_);
}

void CircularTraj::setColor(Color col)
{
  color_ = col;
}


void CircularTraj::setDirection(Direction dir)
{
  direction_ = dir;
}

Direction CircularTraj::getDirection()
{
  return direction_;
}

void CircularTraj::toggleDirection()
{
  switch(direction_)
  {
    case FORWARD:
      direction_ = BACKWARD;
      break;
    case BACKWARD:
    default:
      direction_ = FORWARD;
      break;
  }
}


Point CircularTraj::getNextPoint()
{
  isRunning = true;
  Point pt;
  pt = getNextPoint(current_, color_, direction_);
  return pt;
}

std::string CircularTraj::printPoints(Color col, Direction dir)
{
  std::string cname;
  switch(col)
  {
    case COLOR_RED:
      cname = "Red";
      break;
    case COLOR_BLUE:
    default:
      cname = "Blue";
      break;
  }

  std::stringstream ss;
  ss << "Number of WayPoints : " << pts_.size() << std::endl;
  ss << "Color : " << cname << std::endl;
  if( pts_.size() != 0 )
  {
    Point pt;
    switch(dir)
    {
      case FORWARD:
        ss << "Direction : FORWARD" << std::endl;
        ss << "Points :" << std::endl;
        for(unsigned int i = 0 ; i < pts_.size() ; i++)
        {
          pt = pts_[i].get(col, FORWARD);
          ss << "  [" << i << "] " << pt << std::endl;              
        }
        break;
      case BACKWARD:
      default:
        ss << "Direction : BACKWARD" << std::endl;
        ss << "Points :" << std::endl;
        for(unsigned int i = 0 ; i < pts_.size() ; i++)
        {
          pt = pts_[pts_.size() - 1 - i].get(col, BACKWARD);
          ss << "  [" << /*pts_.size() - 1 - i <<*/ "] " << pt << std::endl;
        }
        break;
    }
  }
  ss << "=========================";
  return ss.str();
}

void CircularTraj::setIndexPreviousWayPoint(unsigned int index, std::vector<WayPoint>::iterator & it)
{
  if(index < pts_.size() + 1)
  {
    it = pts_.begin();
    for( unsigned int i = 0 ; i < index ; i++, it++){ ;}
  }
}

Point CircularTraj::getNextPoint(std::vector<WayPoint>::iterator & it, Color col, Direction dir)
{
  Point pt;
  switch(dir)
  {
    case FORWARD:
      if( it == pts_.end() )
        it = pts_.begin();
      else
      {
        it++;          
        if( it == pts_.end() )
          it = pts_.begin();
      }   
      pt = it->get(col, FORWARD);
      break;
    case BACKWARD:
    default:
      if( it == pts_.begin() )
        it = pts_.end();
      it--;          
      pt = it->get(col, BACKWARD);
      break;
  }
  return pt;
}


