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

/** \ingroup lsl
 * \nonstableyet
 *
 * \class DetectedObject
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
         *  Il construit un DetectedObject vide.
         */
        DetectedObject();

        /** Constructeur par copy.
        */
        DetectedObject(const DetectedObject &);

        /**
         * Modifie le scan associe à l'objet.
         * \param ls le scan associé
         * \remarks Cette méthode fait appel à la méthode calcul de statistique.
         * ainsi, juste après avoir utilisé setScan, il est possible d'accéder
         * aux données statistiques de l'objet ainsi qu'à sa mesure apparente.
         */
        void setScan(lsl::LaserScan ls);

        /**
         * Permet d'accéder au scan sur lequel se base l'objet.
         */
        LaserScan getScan() const;

        /**
         * Permet d'obtenir le centre de gravité de l'objet (ie le centre de gravité de la portion de scan associée à l'objet)
         */
        arp_math::Vector2 getCartesianMean() const;

        /**
         * Permet d'obtenir l'écart type de l'objet (ie l'écart type de la portion de scan associée à l'objet)
         */
        arp_math::Vector2 getCartesianStddev() const;

        /**
         * Permet d'obtenir la distance à l'origine du polaire de l'objet.\n
         * Il s'agit des coordonnées polaires du centre de gravité de la portion de scan associée à l'objet.
         */
        double getApparentRange() const;

        /**
         * Permet d'obtenir l'angle par rapport à l'axe de référence du repère du polaire de l'objet.\n
         * Il s'agit des coordonnées polaires du centre de gravité de la portion de scan associée à l'objet.
         */
        double getApparentTheta() const;


    protected:
        lsl::LaserScan associatedScan;
        double apparentRange;
        double apparentTheta;
        arp_math::Vector2 cartMean;
        arp_math::Vector2 cartStddev;

        /**
         * Calcule les statistiques de la portion de scan associée à l'objet.\n
         * En particulier : le centre de gravité, l'écart type, les coordonnées polaires du centre de gravité.
         */
        void computeStatistics();
};

} // namespace lsl
} // namespace arp_rlu

#endif /* _ARP_RLU_LSL_DETECTEDOBJECT_HPP_ */