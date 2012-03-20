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
    // Prediction

    KFLSysInput input;
    input(0) = 2.0;
    input(1) = -1.0;
    input(2) = 0.;

    double dt = 0.01;

    BFLWrapper::PredictParams predictParams;
    predictParams.odoVelXSigma = 0.001;
    predictParams.odoVelYSigma = 0.001;
    predictParams.odoVelHSigma = 0.01;

    bayesian.predict( input, dt, predictParams );


    //**********************************************
    // Results comparison

    KFLStateVar estim = bayesian.getEstimate();
    KFLStateCov covariance = bayesian.getCovariance();

    //    Log( DEBUG ) << "covariance=\n" << covariance;

    KFLStateVar groundEstim;
    groundEstim(0) = initStateVar(0) + dt * input(0);
    groundEstim(1) = initStateVar(1) + dt * input(1);
    groundEstim(2) = betweenMinusPiAndPlusPi(initStateVar(2) + dt * input(2));

    KFLStateCov groundCov = KFLStateCov::Zero();
    groundCov(0,0) = (sqrt(initStateCov(0,0)) + dt * predictParams.odoVelXSigma)*(sqrt(initStateCov(0,0)) + dt * predictParams.odoVelXSigma);
    groundCov(1,1) = (sqrt(initStateCov(1,1)) + dt * predictParams.odoVelYSigma)*(sqrt(initStateCov(1,1)) + dt * predictParams.odoVelYSigma);
    groundCov(2,2) = (sqrt(initStateCov(2,2)) + dt * predictParams.odoVelHSigma)*(sqrt(initStateCov(2,2)) + dt * predictParams.odoVelHSigma);

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
    // Prediction

    KFLSysInput input;
    input(0) = 0.;
    input(1) = 0.;
    input(2) = 1.;

    double dt = 0.01;

    BFLWrapper::PredictParams predictParams;
    predictParams.odoVelXSigma = 0.001;
    predictParams.odoVelYSigma = 0.001;
    predictParams.odoVelHSigma = 0.01;

    bayesian.predict( input, dt, predictParams );


    //**********************************************
    // Results comparison

    KFLStateVar estim = bayesian.getEstimate();
    KFLStateCov covariance = bayesian.getCovariance();

    //    Log( DEBUG ) << "covariance=\n" << covariance;

    KFLStateVar groundEstim;
    groundEstim(0) = initStateVar(0) + dt * input(0);
    groundEstim(1) = initStateVar(1) + dt * input(1);
    groundEstim(2) = betweenMinusPiAndPlusPi(initStateVar(2) + dt * input(2));

    KFLStateCov groundCov = KFLStateCov::Zero();
    groundCov(0,0) = (sqrt(initStateCov(0,0)) + dt * predictParams.odoVelXSigma)*(sqrt(initStateCov(0,0)) + dt * predictParams.odoVelXSigma);
    groundCov(1,1) = (sqrt(initStateCov(1,1)) + dt * predictParams.odoVelYSigma)*(sqrt(initStateCov(1,1)) + dt * predictParams.odoVelYSigma);
    groundCov(2,2) = (sqrt(initStateCov(2,2)) + dt * predictParams.odoVelHSigma)*(sqrt(initStateCov(2,2)) + dt * predictParams.odoVelHSigma);

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

    KFLSysInput input;
    input(0) = 2.0;
    input(1) = -1.0;
    input(2) = 0.;

    double dt = 0.01;

    BFLWrapper::PredictParams predictParams;
    predictParams.odoVelXSigma = 0.001;
    predictParams.odoVelYSigma = 0.001;
    predictParams.odoVelHSigma   = 0.01;

    for(unsigned int i = 0 ; i < N ; i++)
    {
        bayesian.predict( input, dt, predictParams );
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
    groundCov(0,0) = (sqrt(initStateCov(0,0)) + N * dt * predictParams.odoVelXSigma)*(sqrt(initStateCov(0,0)) + N * dt * predictParams.odoVelXSigma);
    groundCov(1,1) = (sqrt(initStateCov(1,1)) + N * dt * predictParams.odoVelYSigma)*(sqrt(initStateCov(1,1)) + N * dt * predictParams.odoVelYSigma);
    groundCov(2,2) = (sqrt(initStateCov(2,2)) + N * dt * predictParams.odoVelHSigma)*(sqrt(initStateCov(2,2)) + N * dt * predictParams.odoVelHSigma);

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

    KFLSysInput input;
    input(0) = 0.;
    input(1) = 0.;
    input(2) = 1.;

    double dt = 0.01;

    BFLWrapper::PredictParams predictParams;
    predictParams.odoVelXSigma = 0.001;
    predictParams.odoVelYSigma = 0.001;
    predictParams.odoVelHSigma = 0.01;

    for(unsigned int i = 0 ; i < N ; i++)
    {
        bayesian.predict( input, dt, predictParams );
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
    groundCov(0,0) = (sqrt(initStateCov(0,0)) + N * dt * predictParams.odoVelXSigma)*(sqrt(initStateCov(0,0)) + N * dt * predictParams.odoVelXSigma);
    groundCov(1,1) = (sqrt(initStateCov(1,1)) + N * dt * predictParams.odoVelYSigma)*(sqrt(initStateCov(1,1)) + N * dt * predictParams.odoVelYSigma);
    groundCov(2,2) = (sqrt(initStateCov(2,2)) + N * dt * predictParams.odoVelHSigma)*(sqrt(initStateCov(2,2)) + N * dt * predictParams.odoVelHSigma);

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
