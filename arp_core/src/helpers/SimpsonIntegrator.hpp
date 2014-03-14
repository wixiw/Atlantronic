#ifndef __SIMPSON_INTEGRATOR_HPP__
#define __SIMPSON_INTEGRATOR_HPP__

#include "helpers/simpson_integrator.h"

namespace arp_math
{

class SimpsonIntegrator
{
public:
  SimpsonIntegrator();
  ~SimpsonIntegrator();

public:
  void reset(const scalar & value);

public:
  void set(const scalar & age_s, const scalar & derivative);
  scalar get() const;

private:
  SimpsonState * pstate;
};
}

#endif
