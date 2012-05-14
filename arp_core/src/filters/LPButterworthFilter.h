#pragma once

#include "Eigen/Lgsm"

namespace xde
{
namespace math
{
class LPButterworthFilter
{
private:

  /* parameters */
  double samplingPeriod;
  double cutoffFrequency;

  /* cache values */
  double samplingFrequency;
  double omega_A;
  double a[3];
  double b[3];
  bool cacheValues_areUpToDate;
  bool initDone;

  /* filter state */
  Eigen::Displacementd U[2];
  Eigen::Displacementd Y[2];

public:

  inline LPButterworthFilter (const double samplingPeriod, const double cutoffFrequency);
  inline ~LPButterworthFilter ();

  inline void setParameters (const double samplingPeriod, const double cutoffFrequency);
  inline double getSamplingPeriod () const;
  inline double getCutOffFrequency () const;
  inline void initialize (const Eigen::Displacementd& Y0);
  inline bool isInitialized () const;
  inline void reset ();
  inline void computePositionAndVelocity (const Eigen::Displacementd& Uk,
    Eigen::Displacementd& Yk, 
    Eigen::Twistd& Tk);

private:
  inline void updateCacheValues ();
};




LPButterworthFilter::LPButterworthFilter (const double samplingPeriod, const double cutoffFrequency)
{
  /* parameters and cache values */

  this->samplingPeriod = samplingPeriod;
  this->cutoffFrequency = cutoffFrequency;

  cacheValues_areUpToDate = false;
  initDone = false;
}

LPButterworthFilter::~LPButterworthFilter ()
{
  ;
}

void LPButterworthFilter::updateCacheValues ()
{
  if (!cacheValues_areUpToDate) 
  {
    samplingFrequency = 1.0 / samplingPeriod;

    omega_A = tan (M_PI * cutoffFrequency * samplingPeriod);

    a[0] = omega_A * omega_A;
    a[1] = 2.0 * a[0];
    a[2] = a[0];

    b[0] = 1.0 + a[0] + sqrt (2.0) * omega_A;
    b[1] = 2.0 * ( a[0] - 1 );
    b[2] = 1.0 + a[0] - sqrt (2.0) * omega_A;

    cacheValues_areUpToDate = true;
  }
}

void LPButterworthFilter::setParameters (const double samplingPeriod, const double cutoffFrequency)
{
  if ((samplingPeriod != this->samplingPeriod) || (cutoffFrequency != this->cutoffFrequency)) {

    this->samplingPeriod = samplingPeriod;
    this->cutoffFrequency = cutoffFrequency;

    cacheValues_areUpToDate = false;
  }
}

void LPButterworthFilter::initialize (const Eigen::Displacementd& Y0)
{
  U[0] = Y0;
  U[1] = Y0;

  Y[0] = Y0;
  Y[1] = Y0;

  initDone = true;
}

bool LPButterworthFilter::isInitialized () const
{
  return initDone;
}

void LPButterworthFilter::reset ()
{
  initDone = false;
  cacheValues_areUpToDate = false;
}

double LPButterworthFilter::getSamplingPeriod () const
{
  return this->samplingPeriod;
}

double LPButterworthFilter::getCutOffFrequency () const
{
  return this->cutoffFrequency;
}

void LPButterworthFilter::computePositionAndVelocity (const Eigen::Displacementd& Uk,
                                                      Eigen::Displacementd& Yk, 
                                                      Eigen::Twistd& Tk)
{
  updateCacheValues ();

  //*****************************************************************************************
  // Rotation

  Eigen::AngularVelocityd  tmpU[2];
  Eigen::AngularVelocityd  tmpY[2];
  for (unsigned int i = 0; i < 2; ++i) 
  {
    tmpU[i] = ( Uk.getRotation().inverse() * U[i].getRotation() ).log();
    tmpY[i] = ( Uk.getRotation().inverse() * Y[i].getRotation() ).log();
  }

  // XXX does not build with g++ without Eigen::Vector3d
  Eigen::AngularVelocityd tmpVect3 = Eigen::Vector3d((   tmpU[0] * a[1]             // thanks to exponential map
                                                + tmpU[1] * a[2]             // term in front of a[0] is nul
                                                - tmpY[0] * b[1] 
                                                - tmpY[1] * b[2] ) / b[0]);
  Yk.getRotation() = Uk.getRotation() * tmpVect3.exp();

  //*****************************************************************************************
  // Translation
  Yk.getTranslation() = ( Uk.getTranslation()   * a[0] 
                        + U[0].getTranslation() * a[1] 
                        + U[1].getTranslation() * a[2] 
                        - Y[0].getTranslation() * b[1] 
                        - Y[1].getTranslation() * b[2] ) / b[0];

  //*****************************************************************************************
  // Angular velocity
  Tk.getAngularVelocity() =  (Y[0].getRotation ().inverse() * Yk.getRotation()).log() * samplingFrequency;

  //*****************************************************************************************
  // Linear velocity
  Tk.getLinearVelocity() = Yk.getRotation().inverse() * (Yk.getTranslation() - Y[0].getTranslation()) * samplingFrequency;

  //*****************************************************************************************
  // Shift filter state
  U[1] = U[0];
  U[0] = Uk;
  Y[1] = Y[0];
  Y[0] = Yk;
}

}
}
