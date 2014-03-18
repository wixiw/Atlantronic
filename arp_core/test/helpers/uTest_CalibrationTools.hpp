/*
 * uTest_SimpsonIntegrator.hpp
 *
 *  Created on: 14 mars 2014
 *      Author: ard
 */

#ifndef _UTEST_SIMPSONINTEGRATOR_HPP_
#define _UTEST_SIMPSONINTEGRATOR_HPP_

#include <boost/format.hpp>

#include <boost/format.hpp>
#include <stdlib.h>     /* srand, rand */
#include <random>
#include <time.h>       /* time */

#include "helpers/StaticCalibTool.hpp"
#include "helpers/DynamicCalibTool.hpp"

using namespace arp_math;

Eigen::VectorXd generateGaussianVector(unsigned int N, double mean, double var)
{
  /* Generate a new random seed from system time - do this once in your constructor */
  std::default_random_engine generator(time(0));
  std::normal_distribution<double> distribution (mean, var);

  Eigen::VectorXd v = Eigen::VectorXd(N);
  for (int i = 0; i < N; i++)
    v(i) = distribution(generator);
  return v;
}

using boost::format;

BOOST_AUTO_TEST_SUITE( StaticCalibTest )

BOOST_AUTO_TEST_CASE(test_constructor)
{
  StaticCalibrationTool tool;

  BOOST_CHECK( tool.getNbOfSamples() == 0);
  BOOST_CHECK( tool.getEstimatedBias() == 0.);
  BOOST_CHECK( tool.getEstimatedScale() == 1.);
  BOOST_CHECK( tool.getEstimatedNoiseVariance() == 0.);
}

BOOST_AUTO_TEST_CASE(test_few_data)
{
  StaticCalibrationTool tool;

  BOOST_CHECK( !tool.compute() );
  BOOST_CHECK( tool.getNbOfSamples() == 0);
  BOOST_CHECK( tool.getEstimatedBias() == 0.);
  BOOST_CHECK( tool.getEstimatedScale() == 1.);
  BOOST_CHECK( tool.getEstimatedNoiseVariance() == 0.);

  tool.addSample(0., 1.);

  BOOST_CHECK( !tool.compute() );
  BOOST_CHECK( tool.getNbOfSamples() == 1);
  BOOST_CHECK( tool.getEstimatedBias() == 0.);
  BOOST_CHECK( tool.getEstimatedScale() == 1.);
  BOOST_CHECK( tool.getEstimatedNoiseVariance() == 0.);
}

BOOST_AUTO_TEST_CASE(test_trivial)
{
  StaticCalibrationTool tool;

  tool.addSample(0., 3.);
  tool.addSample(0., 3.);

  BOOST_CHECK( tool.compute() );
  BOOST_CHECK( tool.getNbOfSamples() == 2);
  BOOST_CHECK( tool.getEstimatedBias() == 3.);
  BOOST_CHECK( tool.getEstimatedScale() == 1.);
  BOOST_CHECK( tool.getEstimatedNoiseVariance() == 0.);
}

BOOST_AUTO_TEST_CASE(test_full)
{
  StaticCalibrationTool tool;

  tool.addSample(0., 3.);
  tool.addSample(0., 5.);
  tool.addSample(0.,-2.);
  tool.addSample(0., 0.5);
  tool.addSample(0., 4.);
  tool.addSample(0., 0.);

  BOOST_CHECK( tool.compute() );
  BOOST_CHECK( tool.getNbOfSamples() == 6);
  BOOST_CHECK( tool.getEstimatedBias() == 1.75);
  BOOST_CHECK( tool.getEstimatedScale() == 1.);
  BOOST_CHECK( tool.getEstimatedNoiseVariance() == 7.175);
}

BOOST_AUTO_TEST_SUITE_END()


BOOST_AUTO_TEST_SUITE( DynamicCalibTest )

BOOST_AUTO_TEST_CASE(test_constructor)
{
  DynamicCalibrationTool tool;

  BOOST_CHECK( tool.getNbOfSamples() == 0);
  BOOST_CHECK( tool.getEstimatedBias() == 0.);
  BOOST_CHECK( tool.getEstimatedScale() == 0.);
  BOOST_CHECK( tool.getEstimatedNoiseVariance() == 0.);
}

BOOST_AUTO_TEST_CASE(test_few_data)
{
  DynamicCalibrationTool tool;

  BOOST_CHECK( !tool.compute() );
  BOOST_CHECK( tool.getNbOfSamples() == 0);
  BOOST_CHECK( tool.getEstimatedBias() == 0.);
  BOOST_CHECK( tool.getEstimatedScale() == 0.);
  BOOST_CHECK( tool.getEstimatedNoiseVariance() == 0.);

  tool.addSample(0., 1.);

  BOOST_CHECK( !tool.compute() );
  BOOST_CHECK( tool.getNbOfSamples() == 1);
  BOOST_CHECK( tool.getEstimatedBias() == 0.);
  BOOST_CHECK( tool.getEstimatedScale() == 0.);
  BOOST_CHECK( tool.getEstimatedNoiseVariance() == 0.);
}

BOOST_AUTO_TEST_CASE(test_full_without_noise)
{
  for(unsigned int k = 0 ; k < 10 ; ++k)
  {
    unsigned int N = 1000 + (rand() % 10000);
    double g = 1. + ((double)(2*rand()) - RAND_MAX) / RAND_MAX * 0.1;
    double b = ((double)(2*rand()) - RAND_MAX) / RAND_MAX * 50.;

    Eigen::VectorXd meas = Eigen::VectorXd::Random(N);
    Eigen::VectorXd gtruth = g * meas + b * Eigen::VectorXd::Ones(N);

    DynamicCalibrationTool tool;

    for(unsigned int i = 0 ; i < N ; ++i)
      tool.addSample(gtruth(i), meas(i));

    BOOST_CHECK( tool.compute() );
    BOOST_CHECK( tool.getNbOfSamples() == N);
    BOOST_CHECK_CLOSE( tool.getEstimatedBias(), b, 0.00001);
    BOOST_CHECK_CLOSE( tool.getEstimatedScale(), g, 0.00001);
    BOOST_CHECK_SMALL( tool.getEstimatedNoiseVariance(), 0.0000001);
  }
}

BOOST_AUTO_TEST_CASE(test_full_with_noise)
{
  srand(time(NULL));
  for(unsigned int k = 0 ; k < 100 ; ++k)
  {
    unsigned int N = 1000 + (rand() % 10000);
    double g = 1. + ((double)(2*rand()) - RAND_MAX) / RAND_MAX * 0.1;
    double b = ((double)(2*rand()) - RAND_MAX) / RAND_MAX * 50.;
    double var = 0.000001;

    // generate pseudo normal veector
    Eigen::VectorXd noise = generateGaussianVector(N, 0., sqrt(var));
    // extract residual mean
    noise = noise - (noise.sum() / N) * Eigen::VectorXd::Ones(N);
    // compute actual variance
    var = noise.array().square().sum() / (N-1);

    Eigen::VectorXd meas = Eigen::VectorXd::Random(N);
    Eigen::VectorXd gtruth = g * meas + b * Eigen::VectorXd::Ones(N) + noise;

    DynamicCalibrationTool tool;

    for(unsigned int i = 0 ; i < N ; ++i)
      tool.addSample(gtruth(i), meas(i));

    BOOST_CHECK( tool.compute() );
    BOOST_CHECK( tool.getNbOfSamples() == N);
    BOOST_CHECK_CLOSE( tool.getEstimatedBias(), b, 0.1);
    BOOST_CHECK_CLOSE( tool.getEstimatedScale(), g, 0.1);
    BOOST_CHECK_CLOSE( tool.getEstimatedNoiseVariance(), var, 2.);
  }
}

BOOST_AUTO_TEST_SUITE_END()


#endif
