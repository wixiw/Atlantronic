#include "helpers/SimpsonIntegrator.hpp"

using namespace arp_math;

SimpsonIntegrator::SimpsonIntegrator()
  : pstate(0x0)
{
  pstate = simpson_construct_state();
}

SimpsonIntegrator::~SimpsonIntegrator()
{
  simpson_destroy_state(pstate);
}

void SimpsonIntegrator::reset(const scalar & value)
{
  simpson_reset(pstate, value);
}

void SimpsonIntegrator::set(const scalar & age_s, const scalar & derivative)
{
  simpson_set_derivative(pstate, age_s, derivative);
  simpson_compute(pstate);
}

scalar SimpsonIntegrator::get() const
{
    return simpson_get(pstate);
}
