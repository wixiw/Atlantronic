#pragma once

#include "Eigen/Lgsm"

namespace xde
{
  namespace math
  {
    class TwistIntegrator
    {
    private:
      Eigen::Displacementd H_t;
      Eigen::Displacementd H_tm1;
      Eigen::Twistd        T_t;
      Eigen::Twistd        T_tm1;
      bool          initialized;

    public:

      inline void integrate(const double dt);

      inline TwistIntegrator();

      inline ~TwistIntegrator();

      inline void initialize(const Eigen::Displacementd& H_1_0_tm1, const Eigen::Twistd& T_1_0_p_1_r_1_tm1 );

      inline bool isInitialized() const;

      inline void cleanup();

      inline void set_H_1_0_tm1(const Eigen::Displacementd& H_1_0_tm1);

      inline void set_T_1_0_p_1_r_1_tm1(const Eigen::Twistd& T_1_0_p_1_r_1_tm1 );

      inline void set_T_1_0_p_1_r_1_t(const Eigen::Twistd& T_1_0_p_1_r_1_t );

      inline Eigen::Displacementd get_H_1_0_t() const;
    };



    TwistIntegrator::TwistIntegrator () :
    initialized(false)
    {
      ;
    }


    TwistIntegrator::~TwistIntegrator ()
    {
      ;
    }

    void TwistIntegrator::integrate (const double dt)
    {
      const double dt_2 = dt * 0.5;
      const double dt2_8 = dt * dt / 8.0;
      Eigen::Twistd T = (T_tm1 + T_t)*dt_2 - T_tm1.bracket(T_t)*dt2_8;
      H_t = T.exp() * H_tm1;
      H_tm1 = H_t;
    }


    void TwistIntegrator::initialize(const Eigen::Displacementd& H_1_0_tm1, const Eigen::Twistd& T_1_0_p_1_r_1_tm1 )
    {
      T_t = T_1_0_p_1_r_1_tm1;
      this->set_H_1_0_tm1(H_1_0_tm1);
      this->set_T_1_0_p_1_r_1_tm1(T_1_0_p_1_r_1_tm1);
      initialized = true;
    }

    bool TwistIntegrator::isInitialized () const
    {
      return initialized;
    }

    void TwistIntegrator::cleanup()
    {
      initialized = false;
    }

    void TwistIntegrator::set_H_1_0_tm1(const Eigen::Displacementd& H_1_0_tm1)
    {
      H_tm1 = H_1_0_tm1;
    }


    void TwistIntegrator::set_T_1_0_p_1_r_1_tm1(const Eigen::Twistd& T_1_0_p_1_r_1_tm1 )
    {
      T_tm1 = T_1_0_p_1_r_1_tm1;
    }

    void TwistIntegrator::set_T_1_0_p_1_r_1_t(const Eigen::Twistd& T_1_0_p_1_r_1_t )
    {
      T_tm1 = T_t;
      T_t = T_1_0_p_1_r_1_t;
    }


    Eigen::Displacementd TwistIntegrator::get_H_1_0_t() const
    {
      return this->H_t;
    }

  }
}
