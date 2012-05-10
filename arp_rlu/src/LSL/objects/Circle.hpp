/*
 * Circle.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_LSL_CIRCLE_HPP_
#define _ARP_RLU_LSL_CIRCLE_HPP_

#include <math/core>

namespace arp_rlu
{

namespace lsl
{

/*!
 *  \addtogroup lsl
 *  @{
 */

/** \class Circle
 *
 * \brief Circle représente un cercle, à savoir une position dans le plan et un rayon.
 *
 */
class Circle
{
    public:
        /** Constructeur par défault.\n
         *  Il initialise le cercle au centre du repère avec un rayon nul.
         */
        Circle();

        /** Constructeur via des doubles.\n
         *  Il initialise le cercle à partir de trois double.\n
         *  \param _x définit la position selon l'axe x\n
         *  \param _x définit la position selon l'axe h\n
         *  \param _r définit le rayon (valeur par défault = 1.)
         */
        Circle(double _x, double _y, double _r = 1.);

        /** Constructeur via un Vector2 et un double.\n
         *  Il initialise le cercle à partir de trois double.\n
         *  \param _pos définit la position du centre\n
         *  \param _r définit le rayon (valeur par défault = 1.)
         */
        Circle(arp_math::Vector2 _pos, double _r = 1.);

        /**
         * Permet d'accéder à la position selon d'axe x en mètre
         * \return position selon l'axe x en mètre
         */
        double x() const;

        /**
         * Permet d'accéder à la position selon d'axe y en mètre
         * \return position selon l'axe y en mètre
         */
        double y() const;

        /**
         * Permet d'accéder au rayon en mètre
         * \return le rayon en mètre
         */
        double r() const;

        /**
         * Permet de modifier la position selon d'axe x
         * \param position selon l'axe x en mètre
         */
        void x(double _x);

        /**
         * Permet de modifier la position selon d'axe y
         * \param position selon l'axe y en mètre
         */
        void y(double _y);

        /**
         * Permet de modifier le rayon
         * \param rayon en mètre
         */
        void r(double _r);

        /**
         * Permet d'accéder à la position du centre du cercle sous forme d'un Vector2
         * \return position du centre du cercle
         */
        arp_math::Vector2 getPosition() const;

        /**
         * Permet de modifier la position du centre du cercle
         * \param _pos position du centre du cercle
         */
        void setPosition(arp_math::Vector2 _pos);

        /**
         * Permet d'afficher (x, y, r)
         */
        std::string toString();


    protected:
        arp_math::Vector2 cartesianPosition;
        double radius;

};

/*! @} End of Doxygen Groups*/

} // namespace lsl

} // namespace arp_rlu

#endif /* _ARP_RLU_LSL_CIRCLE_HPP_ */
