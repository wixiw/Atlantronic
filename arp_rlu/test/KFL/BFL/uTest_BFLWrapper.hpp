/*
 * uTest_BFLWrapper.hpp
 *
 *  Created on: 22 January 2012
 *      Author: boris
 */

#include "KFL/BFL/BFLWrapper.hpp"

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace kfl;


BOOST_AUTO_TEST_SUITE( unittest_BFLWrapper )

BOOST_AUTO_TEST_CASE( Init_1 )
{
    BFLWrapper bayesian;

    KFLStateVar initStateVar;
    initStateVar(0) = 1.;
    initStateVar(1) = 2.;
    initStateVar(2) = 3.;

    KFLStateCov initStateCov;
    initStateCov << 2.0, 1.0, 0.5,
            1.0, 1.0, 0.0,
            0.5, 0.0, 3.0;

    BFLWrapper::FilterParams params;

    bayesian.init(initStateVar, initStateCov, params);

    KFLStateVar estim = bayesian.getEstimate();
    KFLStateCov covariance = bayesian.getCovariance();

    for(unsigned int i = 0 ; i < 3 ; i++)
    {
        BOOST_CHECK_EQUAL( initStateVar(i), estim(i) );
        for(unsigned int j = 0 ; j < 3 ; j++)
        {
            BOOST_CHECK_EQUAL( initStateCov(i,j), covariance(i,j) );

        }
    }
}

BOOST_AUTO_TEST_CASE( Predict_trans )
{
    BFLWrapper bayesian;

    //**********************************************
    // Initialization

    double sigmaInitialXPosition = 0.2;
    double sigmaInitialYPosition = 0.3;
    double sigmaInitialHeading = 0.1;

    KFLStateVar initStateVar;
    initStateVar << 1., 2., PI/2.;

    KFLStateCov initStateCov = KFLStateCov::Zero();
    initStateCov(0,0) = sigmaInitialXPosition*sigmaInitialXPosition;
    initStateCov(1,1) = sigmaInitialYPosition*sigmaInitialYPosition;
    initStateCov(2,2) = sigmaInitialHeading*sigmaInitialHeading;

    //    Log( DEBUG ) << "initStateVar=" << initStateVar.transpose();
    //    Log( DEBUG ) << "initStateCov=\n" << initStateCov;

    BFLWrapper::FilterParams filterParams;
    filterParams.iekfMaxIt = 10;
    filterParams.iekfInnovationMin = 0.00015;

    bayesian.init(initStateVar, initStateCov, filterParams);


    //**********************************************
    // Prediction

    KFLInputVar input;
    input(0) = 0.5;
    input(1) = -0.1;
    input(2) = 0.;

    double dt = 0.5;

    double odoVelXSigma = 0.15;
    double odoVelYSigma = 0.1;
    double odoVelHSigma = 0.05;

    KFLInputCov cov = KFLInputCov::Identity();
    cov(0,0) = odoVelXSigma * odoVelXSigma;
    cov(1,1) = odoVelYSigma * odoVelYSigma;
    cov(2,2) = odoVelHSigma * odoVelHSigma;

//    Log( DEBUG ) << "********************* Predict_trans *********************************";
//    Log( DEBUG ) << "Predict_trans : state before prediction :\n" << initStateVar.transpose();
//    Log( DEBUG ) << "Predict_trans : cov before prediction :\n" << initStateCov;
//    Log( DEBUG ) << "Predict_trans : input :\n" << input.transpose();
//    Log( DEBUG ) << "Predict_trans : input covariance :\n" << cov;
//    Log( DEBUG ) << "Predict_trans : dt : " << dt;

    bayesian.predict( input, cov, dt );


    //**********************************************
    // Results comparison

    KFLStateVar estim = bayesian.getEstimate();
    KFLStateCov covariance = bayesian.getCovariance();

//    Log( DEBUG ) << "Predict_trans : state after prediction :\n" << estim.transpose();
//    Log( DEBUG ) << "Predict_trans : cov after prediction :\n" << covariance;

    KFLStateVar groundEstim;
    groundEstim(0) = initStateVar(0) + dt * input(0);
    groundEstim(1) = initStateVar(1) + dt * input(1);
    groundEstim(2) = betweenMinusPiAndPlusPi(initStateVar(2) + dt * input(2));

    KFLStateCov groundCov = KFLStateCov::Zero();
    groundCov(0,0) = (sqrt(initStateCov(0,0)) + dt * odoVelXSigma)*(sqrt(initStateCov(0,0)) + dt * odoVelXSigma);
    groundCov(1,1) = (sqrt(initStateCov(1,1)) + dt * odoVelYSigma)*(sqrt(initStateCov(1,1)) + dt * odoVelYSigma);
    groundCov(2,2) = (sqrt(initStateCov(2,2)) + dt * odoVelHSigma)*(sqrt(initStateCov(2,2)) + dt * odoVelHSigma);

    //    Log( DEBUG ) << "groundCov=\n" << groundCov;

    for(unsigned int i = 0 ; i < 3 ; i++)
    {
        BOOST_CHECK_CLOSE( estim(i), groundEstim(i), 1.f );
        for(unsigned int j = 0 ; j < 3 ; j++)
        {
            BOOST_CHECK_CLOSE( covariance(i,j), groundCov(i,j), 1.f );
        }
    }
}

BOOST_AUTO_TEST_CASE( Predict_rot )
{
    BFLWrapper bayesian;

    //**********************************************
    // Initialization

    double sigmaInitialXPosition = 0.2;
    double sigmaInitialYPosition = 0.3;
    double sigmaInitialHeading = 0.1;

    KFLStateVar initStateVar;
    initStateVar << 1., 2., PI/2.;

    KFLStateCov initStateCov = KFLStateCov::Zero();
    initStateCov(0,0) = sigmaInitialXPosition*sigmaInitialXPosition;
    initStateCov(1,1) = sigmaInitialYPosition*sigmaInitialYPosition;
    initStateCov(2,2) = sigmaInitialHeading*sigmaInitialHeading;

    //    Log( DEBUG ) << "initStateVar=" << initStateVar.transpose();
    //    Log( DEBUG ) << "initStateCov=\n" << initStateCov;

    BFLWrapper::FilterParams filterParams;
    filterParams.iekfMaxIt = 10;
    filterParams.iekfInnovationMin = 0.00015;

    bayesian.init(initStateVar, initStateCov, filterParams);


    //**********************************************
    // Prediction

    KFLInputVar input;
    input(0) = 0.;
    input(1) = 0.;
    input(2) = 1.;

    double dt = 0.01;

    double odoVelXSigma = 0.15;
    double odoVelYSigma = 0.1;
    double odoVelHSigma = 0.05;

    KFLInputCov cov = KFLInputCov::Identity();
    cov(0,0) = odoVelXSigma * odoVelXSigma;
    cov(1,1) = odoVelYSigma * odoVelYSigma;
    cov(2,2) = odoVelHSigma * odoVelHSigma;

//    Log( DEBUG ) << "********************* Predict_rot **********************************";
//    Log( DEBUG ) << "Predict_rot : state before prediction :\n" << initStateVar.transpose();
//    Log( DEBUG ) << "Predict_rot : cov before prediction :\n" << initStateCov;
//    Log( DEBUG ) << "Predict_rot : input :\n" << input.transpose();
//    Log( DEBUG ) << "Predict_rot : input covariance :\n" << cov;
//    Log( DEBUG ) << "Predict_rot : dt : " << dt;

    bayesian.predict( input, cov, dt );


    //**********************************************
    // Results comparison

    KFLStateVar estim = bayesian.getEstimate();
    KFLStateCov covariance = bayesian.getCovariance();

//    Log( DEBUG ) << "Predict_rot : state after prediction :\n" << estim.transpose();
//    Log( DEBUG ) << "Predict_rot : cov after prediction :\n" << covariance;

    KFLStateVar groundEstim;
    groundEstim(0) = initStateVar(0) + dt * input(0);
    groundEstim(1) = initStateVar(1) + dt * input(1);
    groundEstim(2) = betweenMinusPiAndPlusPi(initStateVar(2) + dt * input(2));

    KFLStateCov groundCov = KFLStateCov::Zero();
    groundCov(0,0) = (sqrt(initStateCov(0,0)) + dt * odoVelXSigma)*(sqrt(initStateCov(0,0)) + dt * odoVelXSigma);
    groundCov(1,1) = (sqrt(initStateCov(1,1)) + dt * odoVelYSigma)*(sqrt(initStateCov(1,1)) + dt * odoVelYSigma);
    groundCov(2,2) = (sqrt(initStateCov(2,2)) + dt * odoVelHSigma)*(sqrt(initStateCov(2,2)) + dt * odoVelHSigma);

    //    Log( DEBUG ) << "groundCov=\n" << groundCov;

    for(unsigned int i = 0 ; i < 3 ; i++)
    {
        BOOST_CHECK_SMALL( estim(i) - groundEstim(i), 1.e-8 );
        for(unsigned int j = 0 ; j < 3 ; j++)
        {
            BOOST_CHECK_SMALL( covariance(i,j) - groundCov(i,j), 1.e-8 );
        }
    }
}

BOOST_AUTO_TEST_CASE( Multi_Predict_trans )
{
    BFLWrapper bayesian;

    //**********************************************
    // Initialization

    double sigmaInitialPosition = 0.05;
    double sigmaInitialHeading = 0.1;

    KFLStateVar initStateVar;
    initStateVar << 1., 2., PI/2.;

    KFLStateCov initStateCov = KFLStateCov::Zero();
    initStateCov(0,0) = sigmaInitialPosition*sigmaInitialPosition;
    initStateCov(1,1) = sigmaInitialPosition*sigmaInitialPosition;
    initStateCov(2,2) = sigmaInitialHeading*sigmaInitialHeading;

    //    Log( DEBUG ) << "initStateVar=" << initStateVar.transpose();
    //    Log( DEBUG ) << "initStateCov=\n" << initStateCov;

    BFLWrapper::FilterParams filterParams;
    filterParams.iekfMaxIt = 10;
    filterParams.iekfInnovationMin = 0.00015;

    bayesian.init(initStateVar, initStateCov, filterParams);


    //**********************************************
    // Predictions

    unsigned int N = 10;

    KFLInputVar input;
    input(0) = 2.0;
    input(1) = -1.0;
    input(2) = 0.;

    double dt = 0.01;

    double odoVelXSigma = 0.001;
    double odoVelYSigma = 0.001;
    double odoVelHSigma = 0.01;

    KFLInputCov cov = KFLInputCov::Identity();
    cov(0,0) = odoVelXSigma * odoVelXSigma;
    cov(1,1) = odoVelYSigma * odoVelYSigma;
    cov(2,2) = odoVelHSigma * odoVelHSigma;


    for(unsigned int i = 0 ; i < N ; i++)
    {
        bayesian.predict( input, cov, dt );
    }

    //**********************************************
    // Results comparison

    KFLStateVar estim = bayesian.getEstimate();
    KFLStateCov covariance = bayesian.getCovariance();

    //    Log( DEBUG ) << "covariance=\n" << covariance;

    KFLStateVar groundEstim;
    groundEstim(0) = initStateVar(0) + N * dt * input(0);
    groundEstim(1) = initStateVar(1) + N * dt * input(1);
    groundEstim(2) = betweenMinusPiAndPlusPi(initStateVar(2) + N * dt * input(2));

    KFLStateCov groundCov = KFLStateCov::Zero();
    groundCov(0,0) = (sqrt(initStateCov(0,0)) + N * dt * odoVelXSigma)*(sqrt(initStateCov(0,0)) + N * dt * odoVelXSigma);
    groundCov(1,1) = (sqrt(initStateCov(1,1)) + N * dt * odoVelYSigma)*(sqrt(initStateCov(1,1)) + N * dt * odoVelYSigma);
    groundCov(2,2) = (sqrt(initStateCov(2,2)) + N * dt * odoVelHSigma)*(sqrt(initStateCov(2,2)) + N * dt * odoVelHSigma);

    //    Log( DEBUG ) << "groundCov=\n" << groundCov;

    for(unsigned int i = 0 ; i < 3 ; i++)
    {
        BOOST_CHECK_CLOSE( estim(i), groundEstim(i), 1.f );
        for(unsigned int j = 0 ; j < 3 ; j++)
        {
            BOOST_CHECK_CLOSE( covariance(i,j), groundCov(i,j), 1.f );
        }
    }
}

BOOST_AUTO_TEST_CASE( Multi_Predict_rot )
{
    BFLWrapper bayesian;

    //**********************************************
    // Initialization

    double sigmaInitialPosition = 0.05;
    double sigmaInitialHeading = 0.1;

    KFLStateVar initStateVar;
    initStateVar << 1., 2., PI/2.;

    KFLStateCov initStateCov = KFLStateCov::Zero();
    initStateCov(0,0) = sigmaInitialPosition*sigmaInitialPosition;
    initStateCov(1,1) = sigmaInitialPosition*sigmaInitialPosition;
    initStateCov(2,2) = sigmaInitialHeading*sigmaInitialHeading;

    //    Log( DEBUG ) << "initStateVar=" << initStateVar.transpose();
    //    Log( DEBUG ) << "initStateCov=\n" << initStateCov;

    BFLWrapper::FilterParams filterParams;
    filterParams.iekfMaxIt = 10;
    filterParams.iekfInnovationMin = 0.00015;

    bayesian.init(initStateVar, initStateCov, filterParams);


    //**********************************************
    // Predictions

    unsigned int N = 10;

    KFLInputVar input;
    input(0) = 0.;
    input(1) = 0.;
    input(2) = 1.;

    double dt = 0.01;

    double odoVelXSigma = 0.001;
    double odoVelYSigma = 0.001;
    double odoVelHSigma = 0.01;

    KFLInputCov cov = KFLInputCov::Identity();
    cov(0,0) = odoVelXSigma * odoVelXSigma;
    cov(1,1) = odoVelYSigma * odoVelYSigma;
    cov(2,2) = odoVelHSigma * odoVelHSigma;


    for(unsigned int i = 0 ; i < N ; i++)
    {
        bayesian.predict( input, cov, dt );
    }

    //**********************************************
    // Results comparison

    KFLStateVar estim = bayesian.getEstimate();
    KFLStateCov covariance = bayesian.getCovariance();

    //    Log( DEBUG ) << "covariance=\n" << covariance;

    KFLStateVar groundEstim;
    groundEstim(0) = initStateVar(0) + N * dt * input(0);
    groundEstim(1) = initStateVar(1) + N * dt * input(1);
    groundEstim(2) = betweenMinusPiAndPlusPi(initStateVar(2) + N * dt * input(2));

    KFLStateCov groundCov = KFLStateCov::Zero();
    groundCov(0,0) = (sqrt(initStateCov(0,0)) + N * dt * odoVelXSigma)*(sqrt(initStateCov(0,0)) + N * dt * odoVelXSigma);
    groundCov(1,1) = (sqrt(initStateCov(1,1)) + N * dt * odoVelYSigma)*(sqrt(initStateCov(1,1)) + N * dt * odoVelYSigma);
    groundCov(2,2) = (sqrt(initStateCov(2,2)) + N * dt * odoVelHSigma)*(sqrt(initStateCov(2,2)) + N * dt * odoVelHSigma);

    //    Log( DEBUG ) << "groundCov=\n" << groundCov;

    for(unsigned int i = 0 ; i < 3 ; i++)
    {
        BOOST_CHECK_CLOSE( estim(i), groundEstim(i), 1.f );
        for(unsigned int j = 0 ; j < 3 ; j++)
        {
            BOOST_CHECK_CLOSE( covariance(i,j), groundCov(i,j), 1.f );
        }
    }
}

BOOST_AUTO_TEST_CASE( Update_1 )
{
    KFLStateVar groundEstim;
    groundEstim(0) =  0.0;
    groundEstim(1) =  0.0;
    groundEstim(2) =  0.0;

    BFLWrapper bayesian;

    //**********************************************
    // Initialization

    double sigmaInitialPosition = 0.05;
    double sigmaInitialHeading = 0.1;

    KFLStateVar initStateVar;
    initStateVar(0) = groundEstim(0);
    initStateVar(1) = groundEstim(1);
    initStateVar(2) = groundEstim(2);

    KFLStateCov initStateCov = KFLStateCov::Zero();
    initStateCov(0,0) = sigmaInitialPosition*sigmaInitialPosition;
    initStateCov(1,1) = sigmaInitialPosition*sigmaInitialPosition;
    initStateCov(2,2) = sigmaInitialHeading*sigmaInitialHeading;

    //    Log( DEBUG ) << "initStateVar=" << initStateVar.transpose();
    //    Log( DEBUG ) << "initStateCov=\n" << initStateCov;

    BFLWrapper::FilterParams filterParams;
    filterParams.iekfMaxIt = 10;
    filterParams.iekfInnovationMin = 0.00015;

    bayesian.init(initStateVar, initStateCov, filterParams);


    //**********************************************
    // Update

    BFLWrapper::UpdateParams updateParams;
    updateParams.laserRangeSigma = 0.005;
    updateParams.laserThetaSigma = 0.05;


    KFLMeasTarget target;
    KFLMeasVar meas;

    KFLStateVar estim;
    KFLStateCov covariance;

    // First update
    //    Log( DEBUG ) << " ";
    //    Log( DEBUG ) << "******************************";
    //    Log( DEBUG ) << "First update";
    target(0) = 1.5;
    target(1) = 0.;
    //    Log( DEBUG ) << "target:\n" << target;
    meas(0) = sqrt((target(0) - groundEstim(0))*(target(0) - groundEstim(0)) + (target(1) - groundEstim(1))*(target(1) - groundEstim(1)));
    meas(1) = betweenMinusPiAndPlusPi( atan2(target(1) - groundEstim(1), target(0) - groundEstim(0)) - groundEstim(2));
    //    Log( DEBUG ) << "meas:\n" << meas;
    bayesian.update( meas, target, updateParams );

    estim = bayesian.getEstimate();
    covariance = bayesian.getCovariance();
    //    Log( DEBUG ) << "estim:\n" << estim;
    //    Log( DEBUG ) << "covariance:\n" << covariance;

    // Second update
    //    Log( DEBUG ) << " ";
    //    Log( DEBUG ) << "******************************";
    //    Log( DEBUG ) << "Second update";
    target(0) =  0.;
    target(1) =  1.;
    //    Log( DEBUG ) << "target:\n" << target;
    meas(0) = sqrt((target(0) - groundEstim(0))*(target(0) - groundEstim(0)) + (target(1) - groundEstim(1))*(target(1) - groundEstim(1)));
    meas(1) = betweenMinusPiAndPlusPi( atan2(target(1) - groundEstim(1), target(0) - groundEstim(0)) - groundEstim(2));
    //    Log( DEBUG ) << "meas:\n" << meas;
    bayesian.update( meas, target, updateParams );

    estim = bayesian.getEstimate();
    covariance = bayesian.getCovariance();
    //    Log( DEBUG ) << "estim:\n" << estim;
    //    Log( DEBUG ) << "covariance:\n" << covariance;

    // Third update
    //    Log( DEBUG ) << " ";
    //    Log( DEBUG ) << "******************************";
    //    Log( DEBUG ) << "Third update";
    target(0) = -1.5;
    target(1) =  0.;
    //    Log( DEBUG ) << "target:\n" << target;
    meas(0) = sqrt((target(0) - groundEstim(0))*(target(0) - groundEstim(0)) + (target(1) - groundEstim(1))*(target(1) - groundEstim(1)));
    meas(1) = betweenMinusPiAndPlusPi( atan2(target(1) - groundEstim(1), target(0) - groundEstim(0)) - groundEstim(2));
    //    Log( DEBUG ) << "meas:\n" << meas;
    bayesian.update( meas, target, updateParams );

    estim = bayesian.getEstimate();
    covariance = bayesian.getCovariance();
    //    Log( DEBUG ) << "estim:\n" << estim;
    //    Log( DEBUG ) << "covariance:\n" << covariance;

    // Third update
    //    Log( DEBUG ) << " ";
    //    Log( DEBUG ) << "******************************";
    //    Log( DEBUG ) << "Fourth update";
    target(0) =  0.;
    target(1) = -1.;
    //    Log( DEBUG ) << "target:\n" << target;
    meas(0) = sqrt((target(0) - groundEstim(0))*(target(0) - groundEstim(0)) + (target(1) - groundEstim(1))*(target(1) - groundEstim(1)));
    meas(1) = betweenMinusPiAndPlusPi( atan2(target(1) - groundEstim(1), target(0) - groundEstim(0)) - groundEstim(2));
    //    Log( DEBUG ) << "meas:\n" << meas;
    bayesian.update( meas, target, updateParams );

    estim = bayesian.getEstimate();
    covariance = bayesian.getCovariance();
    //    Log( DEBUG ) << "estim:\n" << estim;
    //    Log( DEBUG ) << "covariance:\n" << covariance;


    //**********************************************
    // Results comparison

    estim = bayesian.getEstimate();
    for(unsigned int i = 0 ; i < 3 ; i++)
    {
        BOOST_CHECK_SMALL( estim(i) - groundEstim(i), 0.01 );
        for(unsigned int j = 0 ; j < 3 ; j++)
        {
            if(i == j)
            {
                BOOST_CHECK( covariance(i,j) > 1e-5 );
            }
            else
            {
                BOOST_CHECK_SMALL( covariance(i,j), 1e-16 );
            }
        }
    }
}

BOOST_AUTO_TEST_CASE( Update_2 )
{
    KFLStateVar groundEstim;
    groundEstim(0) =  0.0;
    groundEstim(1) =  0.0;
    groundEstim(2) =  0.0;

    BFLWrapper bayesian;

    //**********************************************
    // Initialization

    double sigmaInitialPosition = 0.05;
    double sigmaInitialHeading = 0.1;

    KFLStateVar initStateVar;
    initStateVar(0) = groundEstim(0) + 0.05;
    initStateVar(1) = groundEstim(1) - 0.05;
    initStateVar(2) = groundEstim(2) - 0.1;

    KFLStateCov initStateCov = KFLStateCov::Zero();
    initStateCov(0,0) = sigmaInitialPosition*sigmaInitialPosition;
    initStateCov(1,1) = sigmaInitialPosition*sigmaInitialPosition;
    initStateCov(2,2) = sigmaInitialHeading*sigmaInitialHeading;

    //    Log( DEBUG ) << "initStateVar=\n" << initStateVar;
    //    Log( DEBUG ) << "initStateCov=\n" << initStateCov;

    BFLWrapper::FilterParams filterParams;
    filterParams.iekfMaxIt = 10;
    filterParams.iekfInnovationMin = 0.00015;

    bayesian.init(initStateVar, initStateCov, filterParams);


    //**********************************************
    // Update

    BFLWrapper::UpdateParams updateParams;
    updateParams.laserRangeSigma = 0.005;
    updateParams.laserThetaSigma = 0.05;


    KFLMeasTarget target;
    KFLMeasVar meas;

    KFLStateVar estim;
    KFLStateCov covariance;

    // First update
    //    Log( DEBUG ) << " ";
    //    Log( DEBUG ) << "******************************";
    //    Log( DEBUG ) << "First update";
    target(0) = 1.5;
    target(1) = 0.;
    //        Log( DEBUG ) << "target:\n" << target;
    meas(0) = sqrt((target(0) - groundEstim(0))*(target(0) - groundEstim(0)) + (target(1) - groundEstim(1))*(target(1) - groundEstim(1)));
    meas(1) = betweenMinusPiAndPlusPi( atan2(target(1) - groundEstim(1), target(0) - groundEstim(0)) - groundEstim(2));
    //        Log( DEBUG ) << "meas:\n" << meas;
    bayesian.update( meas, target, updateParams );

    estim = bayesian.getEstimate();
    covariance = bayesian.getCovariance();
    //    Log( DEBUG ) << "estim:\n" << estim;
    //    Log( DEBUG ) << "covariance:\n" << covariance;

    // Second update
    //    Log( DEBUG ) << " ";
    //        Log( DEBUG ) << "******************************";
    //        Log( DEBUG ) << "Second update";
    target(0) =  0.;
    target(1) =  1.;
    //        Log( DEBUG ) << "target:\n" << target;
    meas(0) = sqrt((target(0) - groundEstim(0))*(target(0) - groundEstim(0)) + (target(1) - groundEstim(1))*(target(1) - groundEstim(1)));
    meas(1) = betweenMinusPiAndPlusPi( atan2(target(1) - groundEstim(1), target(0) - groundEstim(0)) - groundEstim(2));
    //        Log( DEBUG ) << "meas:\n" << meas;
    bayesian.update( meas, target, updateParams );

    estim = bayesian.getEstimate();
    covariance = bayesian.getCovariance();
    //    Log( DEBUG ) << "estim:\n" << estim;
    //    Log( DEBUG ) << "covariance:\n" << covariance;

    // Third update
    //    Log( DEBUG ) << " ";
    //    Log( DEBUG ) << "******************************";
    //    Log( DEBUG ) << "Third update";
    target(0) = -1.5;
    target(1) =  0.0;
    //        Log( DEBUG ) << "target:\n" << target;
    meas(0) = sqrt((target(0) - groundEstim(0))*(target(0) - groundEstim(0)) + (target(1) - groundEstim(1))*(target(1) - groundEstim(1)));
    meas(1) = betweenMinusPiAndPlusPi( atan2(target(1) - groundEstim(1), target(0) - groundEstim(0)) - groundEstim(2));
    //        Log( DEBUG ) << "meas:\n" << meas;
    bayesian.update( meas, target, updateParams );

    estim = bayesian.getEstimate();
    covariance = bayesian.getCovariance();
    //    Log( DEBUG ) << "estim:\n" << estim;
    //    Log( DEBUG ) << "covariance:\n" << covariance;

    // Third update
    //    Log( DEBUG ) << " ";
    //    Log( DEBUG ) << "******************************";
    //    Log( DEBUG ) << "Fourth update";
    target(0) =  0.;
    target(1) = -1.;
    //        Log( DEBUG ) << "target:\n" << target;
    meas(0) = sqrt((target(0) - groundEstim(0))*(target(0) - groundEstim(0)) + (target(1) - groundEstim(1))*(target(1) - groundEstim(1)));
    meas(1) = betweenMinusPiAndPlusPi( atan2(target(1) - groundEstim(1), target(0) - groundEstim(0)) - groundEstim(2));
    //        Log( DEBUG ) << "meas:\n" << meas;
    bayesian.update( meas, target, updateParams );

    estim = bayesian.getEstimate();
    covariance = bayesian.getCovariance();
    //    Log( DEBUG ) << "estim:\n" << estim;
    //    Log( DEBUG ) << "covariance:\n" << covariance;


    //**********************************************
    // Results comparison

    estim = bayesian.getEstimate();
    for(unsigned int i = 0 ; i < 3 ; i++)
    {
        BOOST_CHECK_SMALL( estim(i) - groundEstim(i), 0.01 );
        for(unsigned int j = 0 ; j < 3 ; j++)
        {
            if(i == j)
            {
                BOOST_CHECK( covariance(i,j) > 1e-5 );
            }
            else
            {
                BOOST_CHECK_SMALL( covariance(i,j), 1e-6 );
            }
        }
    }
}


BOOST_AUTO_TEST_CASE( Update_3 )
{
    KFLStateVar groundEstim;
    groundEstim(0) =  0.5;
    groundEstim(1) = -0.1;
    groundEstim(2) =  0.3;

    BFLWrapper bayesian;

    //**********************************************
    // Initialization

    double sigmaInitialPosition = 0.05;
    double sigmaInitialHeading = 0.1;

    KFLStateVar initStateVar;
    initStateVar(0) = groundEstim(0) + 0.05;
    initStateVar(1) = groundEstim(1) - 0.05;
    initStateVar(2) = groundEstim(2) + 0.1;

    KFLStateCov initStateCov = KFLStateCov::Zero();
    initStateCov(0,0) = sigmaInitialPosition*sigmaInitialPosition;
    initStateCov(1,1) = sigmaInitialPosition*sigmaInitialPosition;
    initStateCov(2,2) = sigmaInitialHeading*sigmaInitialHeading;

    //    Log( DEBUG ) << "initStateVar=" << initStateVar.transpose();
    //    Log( DEBUG ) << "initStateCov=\n" << initStateCov;

    BFLWrapper::FilterParams filterParams;
    filterParams.iekfMaxIt = 10;
    filterParams.iekfInnovationMin = 0.00015;

    bayesian.init(initStateVar, initStateCov, filterParams);



    //**********************************************
    // Update

    BFLWrapper::UpdateParams updateParams;
    updateParams.laserRangeSigma = 0.005;
    updateParams.laserThetaSigma = 0.05;


    KFLMeasTarget target;
    KFLMeasVar meas;

    KFLStateVar estim;
    KFLStateCov covariance;

    // First update
    //    Log( DEBUG ) << " ";
    //    Log( DEBUG ) << "******************************";
    //    Log( DEBUG ) << "First update";
    target(0) = 1.5;
    target(1) = 0.;
    //    Log( DEBUG ) << "target:\n" << target;
    meas(0) = sqrt((target(0) - groundEstim(0))*(target(0) - groundEstim(0)) + (target(1) - groundEstim(1))*(target(1) - groundEstim(1)));
    meas(1) = betweenMinusPiAndPlusPi( atan2(target(1) - groundEstim(1), target(0) - groundEstim(0)) - groundEstim(2));
    //    Log( DEBUG ) << "meas:\n" << meas;
    bayesian.update( meas, target, updateParams );

    estim = bayesian.getEstimate();
    covariance = bayesian.getCovariance();
    //    Log( DEBUG ) << "estim:\n" << estim;
    //    Log( DEBUG ) << "covariance:\n" << covariance;

    // Second update
    //    Log( DEBUG ) << " ";
    //    Log( DEBUG ) << "******************************";
    //    Log( DEBUG ) << "Second update";
    target(0) = -1.5;
    target(1) =  1.;
    //    Log( DEBUG ) << "target:\n" << target;
    meas(0) = sqrt((target(0) - groundEstim(0))*(target(0) - groundEstim(0)) + (target(1) - groundEstim(1))*(target(1) - groundEstim(1)));
    meas(1) = betweenMinusPiAndPlusPi( atan2(target(1) - groundEstim(1), target(0) - groundEstim(0)) - groundEstim(2));
    //    Log( DEBUG ) << "meas:\n" << meas;
    bayesian.update( meas, target, updateParams );

    estim = bayesian.getEstimate();
    covariance = bayesian.getCovariance();
    //    Log( DEBUG ) << "estim:\n" << estim;
    //    Log( DEBUG ) << "covariance:\n" << covariance;

    // Third update
    //    Log( DEBUG ) << " ";
    //    Log( DEBUG ) << "******************************";
    //    Log( DEBUG ) << "Third update";
    target(0) = -1.5;
    target(1) = -1.;
    //    Log( DEBUG ) << "target:\n" << target;
    meas(0) = sqrt((target(0) - groundEstim(0))*(target(0) - groundEstim(0)) + (target(1) - groundEstim(1))*(target(1) - groundEstim(1)));
    meas(1) = betweenMinusPiAndPlusPi( atan2(target(1) - groundEstim(1), target(0) - groundEstim(0)) - groundEstim(2));
    //    Log( DEBUG ) << "meas:\n" << meas;
    bayesian.update( meas, target, updateParams );

    estim = bayesian.getEstimate();
    covariance = bayesian.getCovariance();
    //    Log( DEBUG ) << "estim:\n" << estim;
    //    Log( DEBUG ) << "covariance:\n" << covariance;


    //**********************************************
    // Results comparison

    estim = bayesian.getEstimate();
    covariance = bayesian.getCovariance();

    for(unsigned int i = 0 ; i < 3 ; i++)
    {
        BOOST_CHECK_SMALL( estim(i) - groundEstim(i), 0.01 );
    }
}

BOOST_AUTO_TEST_SUITE_END()
