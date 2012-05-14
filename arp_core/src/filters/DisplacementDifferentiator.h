#pragma once

#include "Eigen/Lgsm"

namespace xde
{
namespace math
{
class DisplacementDifferentiator
{
 private:
  Eigen::Displacementd H_1_0_current;
  Eigen::Displacementd H_1_0_precedent;
  Eigen::Twistd        T_1_0_p_1_r_1;
  bool          initialized;

 public:
  
  inline void compute(double dt);
  
  inline DisplacementDifferentiator ();

  inline ~DisplacementDifferentiator ();

  inline void initialize (const Eigen::Displacementd& H_1_0);

  inline bool isInitialized() const;

  inline void cleanup();

  inline void set_H_1_0 (const Eigen::Displacementd& H_1_0);

  inline Eigen::Twistd get_T_1_0_p_1_r_1 () const;
};



DisplacementDifferentiator::DisplacementDifferentiator() :
initialized(false)
{
  ;
}


DisplacementDifferentiator::~DisplacementDifferentiator()
{
  ;
}

void DisplacementDifferentiator::compute(double dt)
{
  double dt_inv = 1.0 / dt;
  
  T_1_0_p_1_r_1.getAngularVelocity() = ( H_1_0_precedent.getRotation().inverse() * H_1_0_current.getRotation() ).log() * dt_inv;
  T_1_0_p_1_r_1.getLinearVelocity()  = H_1_0_current.getRotation().inverse() * (H_1_0_current.getTranslation() - H_1_0_precedent.getTranslation() ) * dt_inv;
  
}


void DisplacementDifferentiator::initialize(const Eigen::Displacementd& H_1_0)
{
  H_1_0_precedent = H_1_0;
  H_1_0_current = H_1_0;
  T_1_0_p_1_r_1 = Eigen::Twistd::Zero();
  initialized = true;
}

bool DisplacementDifferentiator::isInitialized() const
{
  return initialized;
}

void DisplacementDifferentiator::cleanup()
{
  initialized = false;
}

void DisplacementDifferentiator::set_H_1_0 (const Eigen::Displacementd& H_1_0)
{
  H_1_0_precedent = H_1_0_current;
  H_1_0_current = H_1_0;
}


Eigen::Twistd DisplacementDifferentiator::get_T_1_0_p_1_r_1 () const
{
  return this->T_1_0_p_1_r_1;
}

}
}
