#pragma once

#include "Eigen/Lgsm"

namespace xde
{
  namespace math
  {
    template<typename Scalar, int N>
    class RnTrapezoidalIntegrator
    {
    private:
      Eigen::Matrix<Scalar,N,1> x_t;
      Eigen::Matrix<Scalar,N,1> x_tm1;
      Eigen::Matrix<Scalar,N,1> dx_t;
      Eigen::Matrix<Scalar,N,1> dx_tm1;
      bool                      initialized;

    public:

      inline void integrate(const double dt);

      inline RnTrapezoidalIntegrator();

      inline ~RnTrapezoidalIntegrator();

      template<class X, class DX>
      inline void initialize(const Eigen::MatrixBase<X>& x_tm1, const Eigen::MatrixBase<DX>& dx_tm1);

      inline bool isInitialized() const;

      inline void cleanup();

      template<class X>
      inline void set_dx_t(const Eigen::MatrixBase<X>& dx_t );

      inline Eigen::Matrix<Scalar,N,1> get_x_t() const;

    };


    template<typename Scalar, int N>
    RnTrapezoidalIntegrator<Scalar,N>::RnTrapezoidalIntegrator () :
    initialized(false)
    {
      ;
    }

    template<typename Scalar, int N>
    RnTrapezoidalIntegrator<Scalar,N>::~RnTrapezoidalIntegrator ()
    {
      ;
    }

    template<typename Scalar, int N>
    void RnTrapezoidalIntegrator<Scalar,N>::integrate (const double dt)
    {
      x_t = x_tm1 + (dx_tm1 + dx_t) * dt / 2.0;
      
      dx_tm1 = dx_t;
      x_tm1  = x_t;
    }

    template<typename Scalar, int N>
    template<class X, class DX>
    void RnTrapezoidalIntegrator<Scalar,N>::initialize(const Eigen::MatrixBase<X>&  _x_tm1, 
      const Eigen::MatrixBase<DX>& _dx_tm1)
    {
      //EIGEN_STATIC_ASSERT((Eigen::ei_is_same_type<Scalar, typename Eigen::ei_traits<X>::Scalar>::ret),
      //  YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)
      //EIGEN_STATIC_ASSERT((Eigen::ei_is_same_type<Scalar, typename Eigen::ei_traits<DX>::Scalar>::ret),
      //  YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)

      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(X,  N, 1)
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DX, N, 1)

      x_tm1  = _x_tm1;
      dx_tm1 = _dx_tm1;
      initialized = true;
    }

    template<typename Scalar, int N>
    bool RnTrapezoidalIntegrator<Scalar,N>::isInitialized () const
    {
      return initialized;
    }

    template<typename Scalar, int N>
    void RnTrapezoidalIntegrator<Scalar,N>::cleanup()
    {
      initialized = false;
    }

    template<typename Scalar, int N>
    template<class X>
    void RnTrapezoidalIntegrator<Scalar,N>::set_dx_t(const Eigen::MatrixBase<X>& _dx_t )
    {
      //EIGEN_STATIC_ASSERT((Eigen::ei_is_same_type<Scalar, typename Eigen::ei_traits<X>::Scalar>::ret),
      //  YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)

        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(X, N, 1)

      dx_t   = _dx_t;
    }

    template<typename Scalar, int N>
    Eigen::Matrix<Scalar,N,1> RnTrapezoidalIntegrator<Scalar,N>::get_x_t() const
    {
      return this->x_t;
    }

  }
}
