#ifndef __DYNAMIC_CALIBRATION_TOOL_HPP__
#define __DYNAMIC_CALIBRATION_TOOL_HPP__


#include "helpers/ICalibrationTool.hpp"
#include <vector>

namespace arp_math
{

class DynamicCalibrationTool : public ICalibrationTool
{
public:
  DynamicCalibrationTool();

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
  std::vector<double> gtruth;
  double bias;
  double var;
  double scale;
};

}

#endif
