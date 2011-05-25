/*
 * Twist2.cpp
 *
 *  Created on: 12 sept. 2010
 *      Author: boris
 */

#include "Twist2.hpp"

namespace arp_math
{

    Twist2::Twist2(Twist2 & _ref) :
        vitesseTranslation(Vector2(0, 0)), vitesseRotation(0)
    {
        vitesseTranslation = _ref.VTrans();
        vitesseRotation = _ref.VRot();
    }

    Twist2::Twist2(Vector2 _vitesseTranslation, double _vitesseRotation) :
        vitesseTranslation(_vitesseTranslation),
                vitesseRotation(_vitesseRotation)
    {
        ;
    }

    Twist2::~Twist2()
    {
    }

    Vector2 Twist2::VTrans()
    {
        return vitesseTranslation;
    }

    double Twist2::VRot()
    {
        return vitesseRotation;
    }

    void Twist2::VTrans(Vector2 _vitesseTranslation)
    {
        vitesseTranslation = _vitesseTranslation;
    }

    void Twist2::VRot(double _vitesseRotation)
    {
        vitesseRotation = _vitesseRotation;
    }

}
