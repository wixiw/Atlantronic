/*
 * uTest_SimpsonIntegrator.hpp
 *
 *  Created on: 14 mars 2014
 *      Author: ard
 */

#ifndef _UTEST_SIMPSONINTEGRATOR_HPP_
#define _UTEST_SIMPSONINTEGRATOR_HPP_

#include <boost/format.hpp>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "helpers/SimpsonIntegrator.hpp"

using namespace arp_math;

BOOST_AUTO_TEST_CASE( SimpsonIntegrator_constructor )
{
    {
      SimpsonIntegrator si;
      BOOST_CHECK(si.get() == 0.);
      BOOST_CHECK(si.get() == 0.);
    }
}

BOOST_AUTO_TEST_CASE(SimpsonIntegrator_reset)
{
  {
    SimpsonIntegrator si;
    si.reset(0.);
    BOOST_CHECK(si.get() == 0.);
  }

  {
    SimpsonIntegrator si;
    si.reset(1.);
    BOOST_CHECK(si.get() == 1.);
  }

  {
    SimpsonIntegrator si;
    si.reset(-1.034850);
    BOOST_CHECK_CLOSE(si.get(), -1.034850, 0.0001);
  }
}

BOOST_AUTO_TEST_CASE(SimpsonIntegrator_integrate_1)
{
  {
    /*SimpsonIntegrator si;
    si.reset(0.);
    si.set(1.0, 1.);
    BOOST_CHECK_CLOSE(si.get(), 1., 0.0001);*/
  }

  {
    SimpsonIntegrator si;
    si.reset(0.);
    si.set(1.0, 2.);
    BOOST_CHECK_CLOSE(si.get(), 2., 0.0001);
  }

  {
    SimpsonIntegrator si;
    si.reset(4.);
    si.set(1.0, 2.);
    BOOST_CHECK_CLOSE(si.get(), 6., 0.0001);
  }

  {
    SimpsonIntegrator si;
    si.reset(4.);
    si.set(0.5, 2.);
    BOOST_CHECK_CLOSE(si.get(), 5., 0.0001);
  }
}


BOOST_AUTO_TEST_CASE(SimpsonIntegrator_integrate_stationary)
{
  {
    SimpsonIntegrator si;
    si.reset( 4.0 );
    BOOST_CHECK_CLOSE(si.get(), 4.0, 0.0001);

    si.set( 0.5, 0.);
    BOOST_CHECK_CLOSE(si.get(), 4.0, 0.0001);

    si.set( 0.5, 0.);
    BOOST_CHECK_CLOSE(si.get(), 4.0, 0.0001);

    si.set( 0.5, 0.);
    BOOST_CHECK_CLOSE(si.get(), 4.0, 0.0001);
  }
}

BOOST_AUTO_TEST_CASE(SimpsonIntegrator_integrate_linear)
{
  {
    SimpsonIntegrator si;
    si.reset( 4.0 );
    BOOST_CHECK_CLOSE(si.get(), 4.0, 0.0001);

    si.set( 0.5, 2.);
    BOOST_CHECK_CLOSE(si.get(), 5.0, 0.0001);

    si.set( 0.5, 2.);
    BOOST_CHECK_CLOSE(si.get(), 6.0, 0.0001);

    si.set( 0.5, 2.);
    BOOST_CHECK_CLOSE(si.get(), 7.0, 0.0001);
  }
}

BOOST_AUTO_TEST_CASE(SimpsonIntegrator_integrate_cubic)
{
  time_t s = time(NULL);
  srand(s);
  /*std::cout << "seed = " << s << std::endl;
    std::cout << std::endl;*/

  for(unsigned int i = 0 ; i < 100 ; ++i)
  {
    // y'(t) = x(t)
    // x(t) = a*t**2 + b*t + c
    double a =  ((double)(2*rand()) - RAND_MAX) / RAND_MAX * 0.1;
    double b =  ((double)(2*rand()) - RAND_MAX) / RAND_MAX * 100.;
    double c =  ((double)(2*rand()) - RAND_MAX) / RAND_MAX * 10.;
    double d =  ((double)(2*rand()) - RAND_MAX) / RAND_MAX * 10.;

    // choose times
    int us_0   = 2000 + (rand() % 100 - 50);  // between 1950µs and 2500µs
    int us_1   = 2000 + (rand() % 100 - 50);  // between 1950µs and 2500µs
    double t_2 = (double)rand() / RAND_MAX * 90.;
    double t_1 = t_2 + us_1 * 0.000001f;
    double t_0 = t_1 + us_0   * 0.000001f;

    // compute values for specific times
    double y_0 = a * (t_0*t_0*t_0) / 3.;
    y_0       += b * t_0*t_0 / 2.;
    y_0       += c * t_0;
    y_0       += d;

    double y_1 = a * (t_1*t_1*t_1) / 3.;
    y_1       += b * t_1*t_1 / 2.;
    y_1       += c * t_1;
    y_1       += d;

    double y_2 = a * (t_2*t_2*t_2) / 3.;
    y_2       += b * t_2*t_2 / 2.;
    y_2       += c * t_2;
    y_2       += d;


    double x_0 = a * (t_0*t_0);
    x_0       += b * t_0;
    x_0       += c;

    double x_1 = a * (t_1*t_1);
    x_1       += b * t_1;
    x_1       += c;

    double x_2 = a * (t_2*t_2);
    x_2       += b * t_2;
    x_2       += c;

    /*std::cout << "us_0  = " << format("%d") % us_0 << std::endl;
    std::cout << "us_1  = " << format("%d") % us_1 << std::endl;
      std::cout << std::endl;
    std::cout << "t_0  = " << format("%+3.10f") % t_0 << std::endl;
    std::cout << "t_1  = " << format("%+3.10f") % t_1 << std::endl;
    std::cout << "t_2  = " << format("%+3.10f") % t_2 << std::endl;
      std::cout << std::endl;
    std::cout << "tkm1 = " << format("%+3.10f") % (t_1 - t_2) << std::endl;
    std::cout << "tk   = " << format("%+3.10f") % (t_0 - t_1) << std::endl;
      std::cout << std::endl;
    std::cout << "a    = " << format("%+3.10f") % a << std::endl;
    std::cout << "b    = " << format("%+3.10f") % b << std::endl;
    std::cout << "c    = " << format("%+3.10f") % c << std::endl;
      std::cout << std::endl;
    std::cout << "y_0  = " << format("%+3.10f") % y_0 << std::endl;
    std::cout << "y_1  = " << format("%+3.10f") % y_1 << std::endl;
    std::cout << "y_2  = " << format("%+3.10f") % y_2 << std::endl;
      std::cout << std::endl;
    std::cout << "x_0  = " << format("%+3.10f") % x_0 << std::endl;
    std::cout << "x_1  = " << format("%+3.10f") % x_1 << std::endl;
    std::cout << "x_2  = " << format("%+3.10f") % x_2 << std::endl;*/

    struct SimpsonState * pstate = 0x0;
    pstate = (SimpsonState *) malloc(sizeof(SimpsonState));

    // initialize state
    pstate->xk   = x_0;
    pstate->xkm1 = x_1;
    pstate->xkm2 = x_2;
    pstate->ykm1 = y_1;
    pstate->ykm2 = y_2;
    pstate->dtk   = t_0 - t_1;
    pstate->dtkm1 = t_1 - t_2;
    pstate->dtkm2 = 1;

    // compute
    simpson_compute(pstate);

    BOOST_CHECK_CLOSE(simpson_get(pstate), y_0, 0.01);

    free(pstate);
  }
}

BOOST_AUTO_TEST_CASE(SimpsonIntegrator_integrate_multiple)
{
  time_t s = time(NULL);
  srand(s);
  /*std::cout << "seed = " << s << std::endl;
    std::cout << std::endl;*/

  for(unsigned int i = 0 ; i < 100 ; ++i)
  {
    // y'(t) = x(t)
    // x(t) = a*t**2 + b*t + c
    double a =  ((double)(2*rand()) - RAND_MAX) / RAND_MAX * 0.1;
    double b =  ((double)(2*rand()) - RAND_MAX) / RAND_MAX * 100.;
    double c =  ((double)(2*rand()) - RAND_MAX) / RAND_MAX * 10.;
    double d =  ((double)(2*rand()) - RAND_MAX) / RAND_MAX * 10.;

    // choose times
    int us_0   = 2000 + (rand() % 100 - 50);  // between 1950µs and 2500µs
    int us_1   = 2000 + (rand() % 100 - 50);  // between 1950µs and 2500µs
    int us_2   = 2000 + (rand() % 100 - 50);  // between 1950µs and 2500µs
    double t_3 = (double)rand() / RAND_MAX * 90.;
    double t_2 = t_3 + us_2 * 0.000001f;
    double t_1 = t_2 + us_1 * 0.000001f;
    double t_0 = t_1 + us_0   * 0.000001f;

    // compute values for specific times
    double y_0 = a * (t_0*t_0*t_0) / 3.;
    y_0       += b * t_0*t_0 / 2.;
    y_0       += c * t_0;
    y_0       += d;

    double y_1 = a * (t_1*t_1*t_1) / 3.;
    y_1       += b * t_1*t_1 / 2.;
    y_1       += c * t_1;
    y_1       += d;

    double y_2 = a * (t_2*t_2*t_2) / 3.;
    y_2       += b * t_2*t_2 / 2.;
    y_2       += c * t_2;
    y_2       += d;

    double y_3 = a * (t_3*t_3*t_3) / 3.;
    y_3       += b * t_3*t_3 / 2.;
    y_3       += c * t_3;
    y_3       += d;


    double x_0 = a * (t_0*t_0);
    x_0       += b * t_0;
    x_0       += c;

    double x_1 = a * (t_1*t_1);
    x_1       += b * t_1;
    x_1       += c;

    double x_2 = a * (t_2*t_2);
    x_2       += b * t_2;
    x_2       += c;

    double x_3 = a * (t_3*t_3);
    x_3       += b * t_3;
    x_3       += c;

    /*std::cout << "us_0  = " << format("%d") % us_0 << std::endl;
    std::cout << "us_1  = " << format("%d") % us_1 << std::endl;
    std::cout << "us_2  = " << format("%d") % us_2 << std::endl;
      std::cout << std::endl;
    std::cout << "t_0  = " << format("%+3.10f") % t_0 << std::endl;
    std::cout << "t_1  = " << format("%+3.10f") % t_1 << std::endl;
    std::cout << "t_2  = " << format("%+3.10f") % t_2 << std::endl;
    std::cout << "t_3  = " << format("%+3.10f") % t_3 << std::endl;
      std::cout << std::endl;
    std::cout << "tkm2 = " << format("%+3.10f") % (t_2 - t_3) << std::endl;
    std::cout << "tkm1 = " << format("%+3.10f") % (t_1 - t_2) << std::endl;
    std::cout << "tk   = " << format("%+3.10f") % (t_0 - t_1) << std::endl;
      std::cout << std::endl;
    std::cout << "a    = " << format("%+3.10f") % a << std::endl;
    std::cout << "b    = " << format("%+3.10f") % b << std::endl;
    std::cout << "c    = " << format("%+3.10f") % c << std::endl;
      std::cout << std::endl;
    std::cout << "y_0  = " << format("%+3.10f") % y_0 << std::endl;
    std::cout << "y_1  = " << format("%+3.10f") % y_1 << std::endl;
    std::cout << "y_2  = " << format("%+3.10f") % y_2 << std::endl;
    std::cout << "y_3  = " << format("%+3.10f") % y_3 << std::endl;
      std::cout << std::endl;
    std::cout << "x_0  = " << format("%+3.10f") % x_0 << std::endl;
    std::cout << "x_1  = " << format("%+3.10f") % x_1 << std::endl;
    std::cout << "x_2  = " << format("%+3.10f") % x_2 << std::endl;
    std::cout << "x_3  = " << format("%+3.10f") % x_3 << std::endl;*/

    struct SimpsonState * pstate = 0x0;
    pstate = (SimpsonState *) malloc(sizeof(SimpsonState));

    // initialize state
    pstate->xk   = x_1;
    pstate->xkm1 = x_2;
    pstate->xkm2 = x_3;
    pstate->ykm1 = y_2;
    pstate->ykm2 = y_3;
    pstate->dtk   = t_1 - t_2;
    pstate->dtkm1 = t_2 - t_3;
    pstate->dtkm2 = 1;

    // compute
    simpson_compute(pstate);

    BOOST_CHECK_CLOSE(simpson_get(pstate), y_1, 0.01);

    // second integration
    pstate->xk   = x_0;
    pstate->dtk   = t_0 - t_1;

    // compute
    simpson_compute(pstate);

    BOOST_CHECK_CLOSE(simpson_get(pstate), y_0, 0.01);

    free(pstate);
  }
}


#endif /* UTEST_MATH_HPP_ */
