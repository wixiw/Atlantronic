/*
 * DetectedCircle.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_LSL_DETECTEDCIRCLE_HPP_
#define _ARP_RLU_LSL_DETECTEDCIRCLE_HPP_

#include <math/core>

#include <LSL/objects/DetectedObject.hpp>
#include <LSL/objects/Circle.hpp>

namespace arp_rlu
{

namespace lsl
{

/*!
 *  \addtogroup lsl
 *  @{
 */

/** \class DetectedCircle
 *
 * \brief DetectedCircle correspond à la fois à un DetectedObject et à un Circle.
 */
class DetectedCircle : public Circle, public DetectedObject
{
    public:
    /**
     * Constructeur par défault qui appelle les constructeurs par défault de Circle et de DetectedObject.
     */
    DetectedCircle();

    /**
     * Constructeur s'appuyant sur un DetectedObject.\n
     * Les membres relatifs au Circle sont initialisés par défault.
     */
    DetectedCircle(const DetectedObject & ls);

    /**
     * Permet d'obtenir la distance entre centre du cercle et l'origine du repère polaire.\n
     * Il s'agit des coordonnées polaires du centre du cercle.
     * \remarks le repère de référence est le repère polaire du scan, pas le repère cartésien global (table)
     */
    double getApparentCenterRange() const;

    /**
     * Permet d'obtenir l'angle par rapport à l'axe de référence du repère du polaire de l'objet.\n
     * Il s'agit des coordonnées polaires du centre du cercle.
     * \remarks le repère de référence est le repère polaire du scan, pas le repère cartésien global (table)
     */
    double getApparentCenterTheta() const;

    /**
     * Permet d'obtenir la date en seconde pour laquelle les valeurs apparentCenterRange et apparentCenterTheta sont valables.
     */
    double getApparentCenterTime() const;

    /**
     * Permet de définir la distance entre centre du cercle et l'origine du repère polaire.\n
     * Il s'agit des coordonnées polaires du centre du cercle.
     * \remarks le repère de référence est le repère polaire du scan, pas le repère cartésien global (table)
     */
    void setApparentCenterRange( double );

    /**
     * Permet de définir l'angle par rapport à l'axe de référence du repère du polaire de l'objet.\n
     * Il s'agit des coordonnées polaires du centre du cercle.
     * \remarks le repère de référence est le repère polaire du scan, pas le repère cartésien global (table)
     */
    void setApparentCenterTheta( double );

    /**
     * Permet de définir la date en seconde pour laquelle les valeurs apparentCenterRange et apparentCenterTheta sont valables.
     */
    void setApparentCenterTime( double );

    protected:
    double apparentCenterRange;
    double apparentCenterTheta;
    double apparentCenterTime;

};

/*! @} End of Doxygen Groups*/

} // namespace lsl

} // namespace arp_rlu

#endif /* _ARP_RLU_LSL_DETECTEDCIRCLE_HPP_ */
