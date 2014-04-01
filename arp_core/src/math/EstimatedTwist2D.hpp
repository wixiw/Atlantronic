/*
 * EstimatedTwist2D.hpp
 *
 *  Created on: 29 January 2012
 *      Author: Boris
 */

#ifndef _ARP_MATH_ESTIMATEDTWIST2D_HPP_
#define _ARP_MATH_ESTIMATEDTWIST2D_HPP_

#include <math/math.hpp>
#include <math/Twist2D.hpp>
#include "time/ArdTime.hpp"

namespace arp_math
{
class EstimatedTwist2D : public Twist2D
{
    public:
    EstimatedTwist2D(const Twist2D & t = Twist2D());

    Covariance3 cov() const;
    arp_time::ArdAbsoluteTime date() const;
    arp_time::ArdAbsoluteTime& dateRef();

    void cov(const Covariance3 &) ;
    void date(const arp_time::ArdAbsoluteTime &);

    /**
     * Transporte et réduit le EstimatedTwist2D courant dans le nouveau repère
     * définit par la Pose2D du nouveau repère dans l'ancien.
     */
    EstimatedTwist2D transport(const Pose2D & p) const;

    /**
     * Change le repère de projection d'un Twist SANS changer le repère de réduction.\n
     * Reprojète AUSSI la matrice de covariance
     * \li soit T_j_i_p_k_r_k le Twist de j par rapport à i projeté dans k et réduit dans k
     * \li soit T_j_i_p_m_r_k le Twist de j par rapport à i projeté dans m et réduit dans k
     * \li soit H_m_k la pose du repère m dans k
     * Alors :
     * \li T_j_i_p_m_r_k = T_j_i_p_k_r_k.changeProjection( H_m_k );
     * Par exemple, la "vitesse du robot" telle qu'on se la représente mentalement correspond
     * au Twist du robot par rapport à la table, projecté dans la table et réduit dans le robot soit :\n
     * T_robot_table_p_table_r_robot = T_robot_table_p_robot_r_robot.changeProjection( H_robot_table.inverse() );
     */
    EstimatedTwist2D changeProjection(const Pose2D & p) const;

    private:
    Covariance3 covariance;
    arp_time::ArdAbsoluteTime estimationDate;
};
}

#endif /* _ARP_MATH_ESTIMATEDTWIST2D_HPP_ */
