/*
 * EstimatedICRSpeed.hpp
 *
 *  Created on: Nov 29, 2013
 *      Author: ard
 */

#ifndef ESTIMATEDICRSPEED_HPP_
#define ESTIMATEDICRSPEED_HPP_

#include <math/math.hpp>
#include <math/ICRSpeed.hpp>
#include <math/EstimatedTwist2D.hpp>

namespace arp_math
{

class EstimatedICRSpeed: public ICRSpeed
{
    public:
        virtual ~EstimatedICRSpeed();

        EstimatedICRSpeed(const ICRSpeed & t = ICRSpeed());

        Covariance3 cov() const;
        long double date() const;
        long double& dateRef();

        void cov(const Covariance3 &) ;
        void date(const long double &);

        EstimatedTwist2D twist() const ;

    /**
     * Transporte et réduit le EstimatedICRSpeed courant dans le nouveau repère
     * définit par la Pose2D du nouveau repère dans l'ancien.
     */
    EstimatedICRSpeed transport(const Pose2D & p) const;

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
    EstimatedICRSpeed changeProjection(const Pose2D & p) const;

    private:
    Covariance3 covariance;
    long double estimationDate;
};

} /* namespace arp_math */
#endif /* ESTIMATEDICRSPEED_HPP_ */
