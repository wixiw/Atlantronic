#ifndef __ICALIBRATIONTOOL_HPP__
#define __ICALIBRATIONTOOL_HPP__

#include "math/math.hpp"
#include <vector>

namespace arp_math
{

class ICalibrationTool
{
public:
  virtual void reset() = 0;

  virtual void addSample(const double & ground_truth, const double & measure) = 0;
  virtual unsigned int getNbOfSamples() const = 0;

public:
  virtual bool compute() = 0;

public:
  virtual double getEstimatedBias() const = 0;
  virtual double getEstimatedScale() const = 0;
  virtual double getEstimatedNoiseVariance() const = 0;

};

}

#endif
