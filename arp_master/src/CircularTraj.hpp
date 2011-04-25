/*
 * CircularTraj.hpp
 *
 *  Created on: 15 apr. 2011
 *      Author: boris
 */

#ifndef ARP_MASTER_CIRCULARTRAJ_HPP
#define ARP_MASTER_CIRCULARTRAJ_HPP

#include "WayPoint.hpp"

#include <vector>
#include <iostream>


namespace arp_master
{

    /**
    * Circular Trajectory is mainly a std::vector of WayPoint.
    * Each WayPoint embedde information about color and direction.
    * You can construct your CircularTraj with WayPoints colored like you want.
    */
    class CircularTraj
    {
      public:
        /**
        * Construct any empty Circular Trajectory.
        * No copy constructor is available at the moment.
        */
        CircularTraj();

        /**
        * Standard destructor of Circular Trajectory.
        */
        ~CircularTraj();

        /**
        * With this functin, you directly push_back on internal std::vector
        */
        void push_back(WayPoint & wp);


        /**
        * Completly clear the internal std::vector.
        */
        void clear();

        /**
        * Return size of internal vector.
        */
        unsigned int getNumberOfWayPoints() const ;

        /**
        * Get a copy of internal vector.
        */
        std::vector<WayPoint> getWayPointsVector() const;

        /**
        * Maybe the trickyest method of the class.
        * With setIndexPreviousWayPoint, you will define the index of
        * last WayPoint you would like to come.
        * An example will be better to explain :
        * Imagine you have 5 WayPoints in your CircularTraj.
        * You would like to begin with the forth WayPoint, in the forward direction :
        * just call setIndexPreviousWayPoint(2);
        * because 2 is the index for the third WayPoint in the trajectory and in the foreward direction,
        * the index iteration is incremental.
        * An other one example :
        * 7 WayPoints, we would like to begin with the last WayPoint, in backward direction,
        * just call  setIndexPreviousWayPoint(0);
        */
        void setIndexPreviousWayPoint(unsigned int index);

        /**
        * Does nothing if getNextPoint() has been called before because it means the match started.
        */
        void setColor(Color col);

        /**
        * Call it when you want.
        * It will influence te next result of getNextPoint()
        */
        void setDirection(Direction dir);

        /**
        * Usefull to be sure of your direction
        */
        Direction getDirection();

        /**
        * Pretty handy
        */
        void toggleDirection();

        /**
        * Pay attention !!
        * It is NOT a const getter at all !
        * Each call of getNextPoint() will get a new Point.
        * Internaly, an iterator is incremented
        */
        Point getNextPoint();

        /**
        * Choose the color and the direction you would like to see the trajectory.
        * You will have a pretty long string with all informations already formated inside.
        */
        std::string printPoints(Color col, Direction dir);

      protected:
        /**
        * Does not class.
        * Used by the public version of setIndexPreviousWayPoint
        */
        void setIndexPreviousWayPoint(unsigned int index, std::vector<WayPoint>::iterator & it);

        /**
        * Does not class.
        * Used by the public version of getNextPoint() and by printPoints()
        */
        Point getNextPoint(std::vector<WayPoint>::iterator & it, Color col, Direction dir);

      protected:
        /**
        * The famous internal std::vector of WayPoints
        */
        std::vector<WayPoint> pts_;

        /**
        * The iterator on current WayPoint.
        * "current" mean the WayPoint I am trying to reach
        */
        std::vector<WayPoint>::iterator current_;

        /**
        * The current direction
        */
        Direction direction_;
    
        /**
        * The current color
        */
        Color color_;
    
        /**
        * Used to desactivate setColor action when getNextPoint() has been called.
        */
        bool isRunning;
    };


}
#endif
