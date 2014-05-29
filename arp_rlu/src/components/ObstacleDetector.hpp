/*
 * ObstacleDetector.hpp
 *
 *  Created on: 14 mai 2012
 *      Author: ard
 */

#ifndef OBSTACLE_DETECTOR_HPP_
#define OBSTACLE_DETECTOR_HPP_

#include "RluTaskContext.hpp"
#include <math/core>

#include <sensor_msgs/LaserScan.h>
#include "time/ArdTime.hpp"

#include <LSL/LaserScan.hpp>
#include <LSL/objects/Circle.hpp>
#include <LSL/objects/DetectedCircle.hpp>
#include <LSL/filters/MedianFilter.hpp>
#include <LSL/filters/PolarCrop.hpp>
#include <LSL/filters/PolarSegment.hpp>
#include <LSL/filters/CircleIdentif.hpp>
#include <LSL/identificators/TrioCircleIdentif.hpp>
#include <LSL/identificators/DuoCircleIdentif.hpp>

#include <linux/tools/robot_interface.h>

namespace arp_rlu
{

class ObstacleDetector: public RluTaskContext
{
    public:
    ObstacleDetector(const std::string& name, arp_math::Pose2D p_H_hky_robot /*position du hokuyo par rapport auc entre du robot*/);
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
     * Rayon standard du robot adverse
     */
    double opponentRadius;

    /**
     * Paramètres pour la quatrième étape, celle qui consiste à identifier les clusters comme étant des cercles.
     */
    lsl::CircleIdentif::Params cip;

    /**
     * Position du repère hky dans le ropère de référence du robot
     */
    arp_math::Pose2D H_hky_robot;


    //**********************************
    // Ports
    RTT::InputPort<hokuyo_scan> inScan;
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


    arp_time::ArdTimeDelta m_monotonicTimeToRealTime;

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

class RearObstacleDetector: public ObstacleDetector
{
    public:
        RearObstacleDetector(const std::string& name) :
            ObstacleDetector(name, arp_math::Pose2D(0.0, -0.154, M_PI))
        {}
};

class FrontObstacleDetector: public ObstacleDetector
{
    public:
        FrontObstacleDetector(const std::string& name) :
            ObstacleDetector(name, arp_math::Pose2D(-0.051, -0.0, 0))
        {}
};


} /* namespace arp_rlu */
#endif /* OBSTACLE_DETECTOR_HPP_ */
