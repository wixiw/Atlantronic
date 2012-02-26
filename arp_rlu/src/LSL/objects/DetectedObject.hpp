/*
 * DetectedObject.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_LSL_DETECTEDOBJECT_HPP_
#define _ARP_RLU_LSL_DETECTEDOBJECT_HPP_

#include <math/core>

#include <LSL/LaserScan.hpp>

namespace arp_rlu
{


namespace lsl
{

/*!
 *  \addtogroup lsl
 *  @{
 */

/** \class DetectedObject
 *
 * \brief DetectedObject correspond à un objet détecté, c'est à dire à une portion de scan qui correspond à un objet particulier.
 *
 * Un DetectedObject ne dérive pas d'un scan mais il possède une copie de portion de scan.\n
 * A priori, bien qu'il ait été détecté, il n'a pas été identifié comme étant tel ou tel object.
 * En revanche, il est situé dans l'environnement et on peut donc définir une mesure polaire virtuelle
 * correspond au centre de cet objet.
 */
class DetectedObject
{
    public:
        /** Constructeur par défault.
         *  Il construit un DetectedObject vide :\n
         *  * le scan associé est vide\n
         *  * la range apparent vaut 0.\n
         *  * le theta apparent vaut 0.\n
         *  * le centre de gravité est en (0., 0.)\n
         *  * la déviation standard vaut (0., 0.)
         */
        DetectedObject();

        /** Constructeur par copie.
        */
        DetectedObject(const DetectedObject &);

        /** Constructeur par scan.\n
         * Les statistiques de l'objet sont calculées immédiatement.
        */
        DetectedObject(const LaserScan & ls);

        /**
         * Modifie le scan associe à l'objet.
         * \param ls le scan associé
         * \remarks Cette méthode fait appel à la méthode calcul de statistique.
         * ainsi, juste après avoir utilisé setScan, il est possible d'accéder
         * aux données statistiques de l'objet ainsi qu'à sa mesure apparente.
         * \remarks Si les données cartésiennes ne sont pas présentes dans le scan,
         * elles sont calculées via des paramètres par défault : (x,y,h) = (0,0,0)
         */
        void setScan(lsl::LaserScan ls);

        /**
         * Permet d'accéder au scan sur lequel se base l'objet.
         */
        LaserScan getScan() const;

        /**
         * Permet d'obtenir le centre de gravité de l'objet (ie le centre de gravité de la portion de scan associée à l'objet)
         * \remarks le repère de référence est le repère cartésien général (la table), pas le repère du scan
         */
        arp_math::Vector2 getCartesianMean() const;

        /**
         * Permet d'obtenir l'écart type de l'objet (ie l'écart type de la portion de scan associée à l'objet)
         * \remarks le repère de référence est le repère cartésien général (la table), pas le repère du scan
         */
        arp_math::Vector2 getCartesianStddev() const;

        /**
         * Permet d'obtenir la distance à l'origine du polaire de l'objet.\n
         * Il s'agit des coordonnées polaires du centre de gravité de la portion de scan associée à l'objet.
         * \remarks le repère de référence est le repère du scan, pas le repère cartésien global (table)
         */
        double getApparentRange() const;

        /**
         * Permet d'obtenir l'angle par rapport à l'axe de référence du repère du polaire de l'objet.\n
         * Il s'agit des coordonnées polaires du centre de gravité de la portion de scan associée à l'objet.
         * \remarks le repère de référence est le repère du scan, pas le repère cartésien global (table)
         */
        double getApparentTheta() const;

        /**
         * Permet d'obtenir la date en seconde pour laquelle les valeurs apparentRange et apparentTheta sont valables.
         */
        double getApparentTime() const;

        /**
         * Permet d'obtenir la position d'où est vue l'objet.
         * \return un Vector2 contenant une position cartésienne de l'origine du repère polaire par rapport
         * au repère de référence cartésien.
         */
        arp_math::Vector2 getApparentPointOfView() const;

        /**
         * Permet d'obtenir l'angle de vue sous lequel est vue l'objet.
         * \return un double contenant l'angle de vue en radian. Cet angle correspond à l'orientation du repère polaire par rapport
         * au repère de référence cartésien (table).
         */
        double getApparentAngleOfView() const;


    protected:
        lsl::LaserScan associatedScan;
        double apparentRange;
        double apparentTheta;
        double apparentTime;
        arp_math::Vector2 apparentPoV;
        double apparentAoV;
        arp_math::Vector2 cartMean;
        arp_math::Vector2 cartStddev;

        /**
         * Calcule les statistiques de la portion de scan associée à l'objet.\n
         * En particulier : le centre de gravité, l'écart type, les coordonnées polaires du centre de gravité.
         * \remarks Cette méthode à appelée à chaque fois que le scan est changé.
         */
        void computeStatistics();
};
/*! @} End of Doxygen Groups*/

} // namespace lsl


} // namespace arp_rlu

#endif /* _ARP_RLU_LSL_DETECTEDOBJECT_HPP_ */
