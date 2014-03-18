#include "helpers/StaticCalibTool.hpp"
#include <numeric>
#include <algorithm>

using namespace arp_math;

StaticCalibrationTool::StaticCalibrationTool()
{
  reset();
}

void StaticCalibrationTool::reset()
{
  samples.clear();
  bias = 0.;
  var = 0.;
}

double StaticCalibrationTool::getEstimatedBias() const
{
  return bias;
}

double StaticCalibrationTool::getEstimatedScale() const
{
  return 1.;
}

double StaticCalibrationTool::getEstimatedNoiseVariance() const
{
  return var;
}

void StaticCalibrationTool::addSample(const double & ground_truth, const double & measure)
{
  samples.push_back( measure );  // we discard ground_truth because it is assumed to be zero)
}

unsigned int StaticCalibrationTool::getNbOfSamples() const
{
  return samples.size();
}

bool StaticCalibrationTool::compute()
{
  if( getNbOfSamples() < 2)
    return false;

  bias = std::accumulate(samples.begin(), samples.end(), 0.0 ) / (double)samples.size();

  std::vector<double> diff;
  for(std::vector<double>::const_iterator it = samples.begin(); it != samples.end(); ++it)
    diff.push_back( (*it) - bias );

  var = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0) / (double)(samples.size() - 1);

  return true;
}
