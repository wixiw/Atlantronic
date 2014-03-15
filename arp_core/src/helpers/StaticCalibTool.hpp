#ifndef __STATIC_CALIBRATION_TOOL_HPP__
#define __STATIC_CALIBRATION_TOOL_HPP__

#include "helpers/ICalibrationTool.hpp"
#include <vector>

namespace arp_math
{

class StaticCalibrationTool : public ICalibrationTool
{
public:
  StaticCalibrationTool();

public:
  void reset();

  void addSample(const double & ground_truth, const double & measure);
  unsigned int getNbOfSamples() const;

public:
  bool compute();

public:
  double getEstimatedBias() const;
  double getEstimatedScale() const;
  double getEstimatedNoiseVariance() const;

private:
  std::vector<double> samples;
  double bias;
  double var;
};

}

#endif
