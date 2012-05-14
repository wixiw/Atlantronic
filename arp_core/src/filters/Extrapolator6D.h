#ifndef _LGSM_EXTRAPOLATOR_6D_H_
#define _LGSM_EXTRAPOLATOR_6D_H_

#include "Eigen/Lgsm"

namespace xde 
{
  namespace math
  {
    class Extrapolator6D
    {
    public:
      // inputs
      void setParameters(const Eigen::Displacementd& H_1_0_t0, const Eigen::Twistd& T_1_0_p_1_r_1_t0);
      void set_H_1_0_t0(const Eigen::Displacementd& H_1_0_t0);
      void set_T_1_0_p_1_r_1_t0(const Eigen::Twistd& T_1_0_p_1_r_1_t0);

      // outputs
      const Eigen::Twistd& get_T_1_0_p_1_r_1_t0() const { return T_1_0_p_1_r_1_t0; }
      const Eigen::Displacementd& get_H_1_0_t0() const { return H_1_0_t0; }
      Eigen::Displacementd get_H_1_0(double tau);

    private:
      /* Parameters */
      Eigen::Displacementd H_1_0_t0;
      Eigen::Twistd T_1_0_p_1_r_1_t0;
    };

    void Extrapolator6D::setParameters(const Eigen::Displacementd& H_1_0_t0, const Eigen::Twistd& T_1_0_p_1_r_1_t0)
    {
      this->H_1_0_t0 = H_1_0_t0;
      this->T_1_0_p_1_r_1_t0 = T_1_0_p_1_r_1_t0;
    }

    void Extrapolator6D::set_H_1_0_t0(const Eigen::Displacementd& H_1_0_t0)
    {
      this->H_1_0_t0 = H_1_0_t0;
    }

    void Extrapolator6D::set_T_1_0_p_1_r_1_t0(const Eigen::Twistd& T_1_0_p_1_r_1_t0)
    {
      this->T_1_0_p_1_r_1_t0 = T_1_0_p_1_r_1_t0;
    }

    Eigen::Displacementd Extrapolator6D::get_H_1_0(double tau)
    {
      Eigen::AngularVelocityd tmpVect3 = T_1_0_p_1_r_1_t0.getAngularVelocity();
      tmpVect3 *= tau;
      return Eigen::Displacementd(
        H_1_0_t0.getTranslation() + tau * (H_1_0_t0.getRotation() * T_1_0_p_1_r_1_t0.getLinearVelocity()),
        H_1_0_t0.getRotation() * tmpVect3.exp()
        );
    }
  } // namespace xde
}
#endif /* _LGSM_EXTRAPOLATOR_6D_H_ */

