/*
 * ActuatorsFrameGlade2014.hpp
 *
 *  Created on: Apr 15, 2014
 *      Author: ard
 */

#ifndef ACTUATORSFRAMEGLADE2014_HPP_
#define ACTUATORSFRAMEGLADE2014_HPP_

#include <gtk/gtk.h>
#include <string>

namespace arp_hml
{

class CannonFrame
{
    public:
        enum eCanonSide
        {
            LEFT=0,
            RIGHT=1
        };

        enum eImageId
        {
            IMG_NO_BALL=0,
            IMG_BALL,
            IMG_STOCKER_LOADING,
            IMG_STOCKER_IDLE,
            IMG_STOCKER_UNLOADING,
            IMG_FINGER_DOWN,
            IMG_FINGER_UP,
            IMG_FINGER_ARMED,
            IMG_SHOOTING,
            IMG_MAX_ID
        };

        enum eStockerPosition
        {
            LOADING,
            IDLE,
            UNLOADING
        };

        enum eFingerPosition
        {
            DOWN,
            UP,
            ARMED,
            SHOOTING
        };

        CannonFrame();
        ~CannonFrame();

        bool init(int argc, char **argv);
        bool spin();
        void shutDown();

        void setNumberOfBallsInCanon(eCanonSide side, int nbBalls);
        void setStockerPosition(eCanonSide side, enum eStockerPosition pos);
        void setFingerPosition(eCanonSide side, enum eFingerPosition pos);

    protected:
        bool initCannon(eCanonSide side, GtkBuilder *builder);

        GtkImage* m_Balls[2][3];
        GtkImage* m_Fingers[2];
        GtkImage* m_Stockers[2];

        std::string m_ImagesPath[IMG_MAX_ID];
        std::string m_PackagePath;
};
} /* namespace arp_hml */
#endif /* ACTUATORSFRAMEGLADE2014_HPP_ */
