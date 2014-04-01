/*
 * SimpleLocalizator.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: willy
 */

#include <iomanip>
#include "SimpleLocalizator.hpp"
#include <rtt/Component.hpp>
#include <ros/ros.h>

using namespace arp_rlu;
using namespace arp_math;
using namespace arp_time;
using namespace RTT;
using namespace std;


ORO_LIST_COMPONENT_TYPE( arp_rlu::SimpleLocalizator)

SimpleLocalizator::SimpleLocalizator(const std::string& name) :
        RluTaskContext(name), propMaxReliableBadOdoTransStddev(0.10), propMaxReliableBadOdoRotStddev(deg2rad(12.)), currentState(
                STOPPED), currentMode(ODO_ONLY), currentQuality(LOST),
                m_defaultInitCovariance(Vector3(0.020 * 0.020, 0.020 * 0.020, deg2rad(1.) * deg2rad(1.)).asDiagonal())
{
    createOrocosInterface();
    attrLastTime = getAbsoluteTime();
}

bool SimpleLocalizator::configureHook()
{
    if (!RluTaskContext::configureHook())
        return false;

    return true;
}

void SimpleLocalizator::updateHook()
{
    ArdAbsoluteTime newTime;
    if (RTT::NewData != inTime.readNewest(newTime))
    {
        LOG( Error ) << "No new data in inTime port : updateHook should not be externally trigger => return"
                << endlog();
        return;
    }

    EstimatedTwist2D T_odo_table_p_odo_r_odo;
    if (RTT::NewData != inTwistOdo.read(T_odo_table_p_odo_r_odo))
    {
        LOG( Error ) << "No new data in inTwistOdo port : updateHook should not be externally trigger => return"
                << endlog();
        return;
    }

//    EstimatedPose2D pose = MathFactory::createEstimatedPose2D(x, y, theta, initDate, defaultInitCovariance);
    // Si on a reçu un cap provenant de l'extérieur, alors on l'applique

    // On estime la position par intégration de l'ICRSpeed courrant;
    ArdTimeDelta dt = getTimeDelta(attrLastTime, newTime);
    attrLastTime = newTime;
    vxIntegrator.set(dt, T_odo_table_p_odo_r_odo.vx());
    vyIntegrator.set(dt, T_odo_table_p_odo_r_odo.vy());
    vhIntegrator.set(dt, T_odo_table_p_odo_r_odo.vh());

    double externalHeading;
    if (RTT::NewData == inTrueHeading.readNewest(externalHeading))
    {
        vhIntegrator.reset(externalHeading);
        LOG( Info ) << "Heading reseted to true value : " << externalHeading << endlog();
    }

    // Si on a reçu une pose provenant de l'extérieur, alors on l'applique
    Pose2D externalPose;
    if (RTT::NewData == inTruePose.readNewest(externalPose))
    {
        currentState = RUNNING;
        vxIntegrator.reset(externalPose.x());
        vyIntegrator.reset(externalPose.y());
        vhIntegrator.reset(externalPose.h());
        LOG( Info ) << "Pose reseted to true value : " << externalPose.toString() << endlog();
    }

    // On construit la Pose finale
    EstimatedPose2D estim_H_robot_table =
            MathFactory::createEstimatedPose2D(
                            vxIntegrator.get(),
                            vyIntegrator.get(),
                            vhIntegrator.get(),
                            newTime,
                            m_defaultInitCovariance //TODO calculer la  vraie covariance
                            );

    //On ne publie que si on tourne, sinon il a pu se passer n'importe quoi
    if (currentState == RUNNING)
    {
        outPose.write(estim_H_robot_table);
    }

    //shall be after the pose publication
    updateLocalizationStates();
}

void SimpleLocalizator::ooSwitchToRedConfig()
{
}

void SimpleLocalizator::ooSwitchToYellowConfig()
{
}

void SimpleLocalizator::updateLocalizationStates()
{
    EstimatedPose2D estim_H_robot_table = outPose.getLastWrittenValue();
    Covariance3 cov = estim_H_robot_table.cov();

    bool reliabilityOdo = true;
    //TODO workaround WLA : desactivation de check de qualite odo:
//    reliabilityOdo = reliabilityOdo
//            && (cov(0, 0) < propMaxReliableBadOdoTransStddev * propMaxReliableBadOdoTransStddev);
//    reliabilityOdo = reliabilityOdo
//            && (cov(1, 1) < propMaxReliableBadOdoTransStddev * propMaxReliableBadOdoTransStddev);
//    reliabilityOdo = reliabilityOdo && (cov(2, 2) < propMaxReliableBadOdoRotStddev * propMaxReliableBadOdoRotStddev);

    // Mode
    currentMode = ODO_ONLY;

    // Quality
    if (reliabilityOdo && currentState == RUNNING)
    {
        currentQuality = GOOD;
    }
    else
    {
        currentQuality = LOST;
    }

    outLocalizationState.write(currentState);
    outLocalizationMode.write(currentMode);

    if (currentState == RUNNING)
    {
        outLocalizationQuality.write(currentQuality);
    }
    else
    {
        outLocalizationQuality.write(LOST);
    }
}

bool SimpleLocalizator::halt()
{
    currentState = STOPPED;
    return true;
}

bool SimpleLocalizator::resume()
{
    currentState = RUNNING;
    return true;
}

std::string SimpleLocalizator::getInfo()
{
    std::stringstream ss;
    ss << "***************************************************************************************************"
            << std::endl;
    ss << "Localization";
    ss << " - State: " << LocalizationStateNames[currentState];
    ss << " - Mode: " << LocalizationModeNames[currentMode];
    ss << " - Quality: " << LocalizationQualityNames[currentQuality];

    if (currentState == RUNNING)
    {
        EstimatedPose2D estim_H_robot_table = outPose.getLastWrittenValue();
        ss << "Estimate : " << estim_H_robot_table.toString() << std::endl;
    }
    else
    {
        ss << "Estimate : None" << std::endl;
    }

    return ss.str();
}

void SimpleLocalizator::createOrocosInterface()
{

    addPort("inOdo", inTwistOdo).doc(
            "Estimation of T_odo_table_p_odo_r_odo : Twist of odo reference frame relative to table frame, reduced and expressed in odo reference frame.\n It is an EstimatedTwist2D, so it contains Twist, estimation date (in sec) and covariance matrix.");

    addPort("inGyroAngle", inGyroAngle).doc("Angular position from the Gyrometer");

    addPort("inTrueHeading",inTrueHeading)
            .doc("Perfect external heading");

    addPort("inTruePose",inTruePose)
            .doc("Perfect external Pose");

    addPort("outPose", outPose).doc(
            "Last estimation of H_robot_table.\n It is an EstimatedPose2D, so it contains Pose2D, estimation date (in sec) and covariance matrix.");

    std::stringstream ssLocState;
    ssLocState << "Localization state :" << std::endl;
    ssLocState << " [*] 0 : STOPPED  if Localization is halt" << std::endl;
    ssLocState << " [*] 1 : RUNNING if Localization is currently running" << std::endl;
    addPort("outLocalizationState", outLocalizationState).doc(ssLocState.str());

    std::stringstream ssLocMode;
    ssLocMode << "Localization mode :" << std::endl;
    ssLocMode << " [*] 0 : ODO_ONLY  if Localization is using odometry only" << std::endl;
    ssLocMode << " [*] 1 : SMOOTH if Localization is using odometry with large gains for Laser" << std::endl;
    ssLocMode << " [*] 2 : FUSION if Localization is using both odometry and laser" << std::endl;
    addPort("outLocalizationMode", outLocalizationMode).doc(ssLocMode.str());

    std::stringstream ssLocQuality;
    ssLocQuality << "Localization quality :" << std::endl;
    ssLocQuality << " [*] 0 : LOST  if Localizator is lost" << std::endl;
    ssLocQuality << " [*] 1 : BAD if Localization has big covariance" << std::endl;
    ssLocQuality << " [*] 2 : GOOD if Localization has small covariance" << std::endl;
    addPort("outLocalizationQuality", outLocalizationQuality).doc(ssLocQuality.str());

    addOperation("ooHalt", &SimpleLocalizator::halt, this, OwnThread).doc("halt localization");

    addOperation("ooResume", &SimpleLocalizator::resume, this, OwnThread).doc("resume localization");

    addOperation("ooSwitchToRedConfig", &SimpleLocalizator::ooSwitchToRedConfig, this, OwnThread).doc(
            "Définit les balises pour le départ Red");

    addOperation("ooSwitchToYellowConfig", &SimpleLocalizator::ooSwitchToYellowConfig, this, OwnThread).doc(
            "Définit les balises pour le départ Yellow");

    addProperty("propMaxReliableBadOdoTransStddev", propMaxReliableBadOdoTransStddev).doc(
            "Threshold on translation for bad odometry detection (unit is meter)");

    addProperty("propMaxReliableBadOdoRotStddev", propMaxReliableBadOdoRotStddev).doc(
            "Threshold on rotation variance for bad odometry detection (unit is radian)");
}
