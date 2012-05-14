#pragma once

#include "lgsm/lgsm"

namespace xde
{
  namespace math
  {
    template<typename Scalar, int N>
    class RnRegressiveDerivator
    {
    private:
      Eigen::Matrix<Scalar,N,1> x_t;
      Eigen::Matrix<Scalar,N,1> x_tm1;
      Eigen::Matrix<Scalar,N,1> dx_t;
      bool                      initialized;

    public:

      inline void derivate(const double dt);

      inline RnRegressiveDerivator();

      inline ~RnRegressiveDerivator();

      template<class X>
      inline void initialize(const Eigen::MatrixBase<X>& x_tm1);

      inline bool isInitialized() const;

      inline void cleanup();

      template<class X>
      inline void set_x_t(const Eigen::MatrixBase<X>& x_t );

      inline Eigen::Matrix<Scalar,N,1> get_dx_t() const;

    };



    template<typename Scalar, int N>
    RnRegressiveDerivator<Scalar,N>::RnRegressiveDerivator () :
    initialized(false)
    {
      ;
    }

    template<typename Scalar, int N>
    RnRegressiveDerivator<Scalar,N>::~RnRegressiveDerivator ()
    {
      ;
    }

    template<typename Scalar, int N>
    void RnRegressiveDerivator<Scalar,N>::derivate (const double dt)
    {
      dx_t = ( x_t - x_tm1) / dt;
      x_tm1  = x_t;
    }

    template<typename Scalar, int N>
    template<class X>
    void RnRegressiveDerivator<Scalar,N>::initialize(const Eigen::MatrixBase<X>&  _x_tm1)
    {
      EIGEN_STATIC_ASSERT((Eigen::ei_is_same_type<Scalar, typename Eigen::ei_traits<X>::Scalar>::ret),
        YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)

      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(X,  N, 1)

      x_tm1  = _x_tm1;
      initialized = true;
    }

    template<typename Scalar, int N>
    bool RnRegressiveDerivator<Scalar,N>::isInitialized () const
    {
      return initialized;
    }

    template<typename Scalar, int N>
    void RnRegressiveDerivator<Scalar,N>::cleanup()
    {
      initialized = false;
    }

    template<typename Scalar, int N>
    template<class X>
    void RnRegressiveDerivator<Scalar,N>::set_x_t(const Eigen::MatrixBase<X>& _x_t )
    {
      EIGEN_STATIC_ASSERT((Eigen::ei_is_same_type<Scalar, typename Eigen::ei_traits<X>::Scalar>::ret),
        YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)

        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(X, N, 1)

      x_t   = _x_t;
    }

    template<typename Scalar, int N>
    Eigen::Matrix<Scalar,N,1> RnRegressiveDerivator<Scalar,N>::get_dx_t() const
    {
      return this->dx_t;
    }

  }
}
