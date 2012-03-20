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
    predictParams.odoVelHSigma   = 0.01;

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

BOOST_AUTO_TEST_SUITE_END()
