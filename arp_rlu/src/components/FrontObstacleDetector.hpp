/*
 * FrontObstacleDetector.hpp
 *
 *  Created on: 14 mai 2012
 *      Author: ard
 */

#ifndef FrontObstacleDetector_HPP_
#define FrontObstacleDetector_HPP_

#include "RluTaskContext.hpp"
#include <math/core>

#include <sensor_msgs/LaserScan.h>

#include <LSL/LaserScan.hpp>
#include <LSL/objects/Circle.hpp>
#include <LSL/objects/DetectedCircle.hpp>
#include <LSL/filters/MedianFilter.hpp>
#include <LSL/filters/PolarCrop.hpp>
#include <LSL/filters/PolarSegment.hpp>
#include <LSL/filters/CircleIdentif.hpp>
#include <LSL/identificators/TrioCircleIdentif.hpp>
#include <LSL/identificators/DuoCircleIdentif.hpp>

namespace arp_rlu
{

class FrontObstacleDetector: public RluTaskContext
{
    public:
    FrontObstacleDetector(const std::string& name);
    bool configureHook();
    void updateHook();

    protected:

    //**********************************
    // Params

    /**
     * Paramètres pour la première étape du traitement, à savoir un filtrage médian.
     */
    lsl::MedianFilter::Params mfp;

    /**
     * Paramètres pour la deuxième étape du traitement, à savoir un crop polaire qui permet de s'affranchir des mesures trop lointaines.
     */
    lsl::PolarCrop::Params pcp;

    /**
     * Paramètres pour la troisième étape, celle qui consiste à identifier des clusters de mesures succeptibles d'être des balises.
     */
    lsl::PolarSegment::Params psp;

    /**
     * Nombre minimal de point pour qu'un DetectedObject soit considéré comme une balise potentielle.
     */
    unsigned int minNbPoints;

    /**
     * Ecart type max des mesures d'un DetectObject pour le considérer comme balise potentielle
     */
    double cartStddevMax;

    /**
     * Paramètres pour la quatrième étape, celle qui consiste à identifier les clusters comme étant des cercles.
     */
    lsl::CircleIdentif::Params cip;

    /**
     * Définit la zone accessible sur la table
     */
    double xMinAccessible;

    /**
     * Définit la zone accessible sur la table
     */
    double xMaxAccessible;

    /**
     * Définit la zone accessible sur la table
     */
    double yMinAccessible;

    /**
     * Définit la zone accessible sur la table
     */
    double yMaxAccessible;

    /**
     * Position du repère hky dans le ropère de référence du robot
     */
    arp_math::Pose2D H_hky_robot;


    //**********************************
    // Ports
    RTT::InputPort<sensor_msgs::LaserScan> inScan;
    RTT::InputPort<arp_math::EstimatedPose2D> inPose;
    RTT::OutputPort< std::vector<arp_math::Vector2> > outObstacles;


    //*****************************************************
    // Operations
    virtual std::string coGetPerformanceReport();


    void createOrocosInterface();


    /**
     * Les clusters détectés.\n
     * Ils n'ont pas (encore) été identifiés comme des cercles.
     */
    std::vector< lsl::DetectedObject > detectedObjects;

    /**
     * Les cercles détectés dans la zone cartésienne définies par xMin, xMax, yMin et yMax.
     */
    std::vector< lsl::DetectedCircle > detectedCircles;

    /**
     * Les obstacles détectés sur le terrain.\n
     * Les bornes cartésiennes de la zone de détection sont définies par xMinObstacle, xMaxObstacle, yMinOBstacle et yMaxObstacle
     */
    std::vector< arp_math::Vector2 > detectedObstacles;


    long double m_monotonicTimeToRealTime;

    arp_core::StatTimer mfTimer;
    arp_core::StatTimer pcTimer;
    arp_core::StatTimer clTimer;
    arp_core::StatTimer cartTimer;
    arp_core::StatTimer psTimer;
    arp_core::StatTimer fiTimer;
    arp_core::StatTimer ciTimer;
    arp_core::StatTimer obsTimer;
    arp_core::StatTimer globalTimer;

};

} /* namespace arp_rlu */
#endif /* FrontObstacleDetector_HPP_ */
