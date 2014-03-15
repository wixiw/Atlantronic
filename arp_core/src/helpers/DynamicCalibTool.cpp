#include "helpers/DynamicCalibTool.hpp"
#include <Eigen/SVD>

using namespace arp_math;

DynamicCalibrationTool::DynamicCalibrationTool()
{
  reset();
}

void DynamicCalibrationTool::reset()
{
  samples.clear();
  gtruth.clear();
  bias = 0.;
  var = 0.;
  scale = 0.;
}

double DynamicCalibrationTool::getEstimatedBias() const
{
  return bias;
}

double DynamicCalibrationTool::getEstimatedScale() const
{
  return scale;
}

double DynamicCalibrationTool::getEstimatedNoiseVariance() const
{
  return var;
}

void DynamicCalibrationTool::addSample(const double & ground_truth, const double & measure)
{
  samples.push_back( measure );
  gtruth.push_back( ground_truth );
}

unsigned int DynamicCalibrationTool::getNbOfSamples() const
{
  return samples.size();
}

bool DynamicCalibrationTool::compute()
{
  unsigned int N = getNbOfSamples();
  if(N < 2)
    return false;

  // solve least-square A.x = W
  Eigen::MatrixXd A = Eigen::MatrixXd::Ones(N, 2);
  Eigen::VectorXd W = Eigen::VectorXd(N);
  for(unsigned int i = 0 ; i < N ; ++i)
  {
    A(i,0) = samples[i];
    W(i)   = gtruth[i];
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Vector2d x = svd.solve(W);

  scale = x(0);
  bias  = x(1);

  Eigen::VectorXd noise = scale * A.col(0) + bias * Eigen::VectorXd::Ones(N) - W;
  var = (noise - (noise.sum() / N) * Eigen::VectorXd::Ones(N)).array().square().sum() / (N-1);

  return true;
}
