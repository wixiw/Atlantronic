#pragma once

#include "Eigen/Lgsm"

namespace xde
{
  namespace math
  {
    template<typename Scalar, int N>
    class RnSimpsonIntegrator
    {
    private:
      Eigen::Matrix<Scalar,N,1> x_t;
      Eigen::Matrix<Scalar,N,1> x_tm1;
      Eigen::Matrix<Scalar,N,1> x_tm2;
      Eigen::Matrix<Scalar,N,1> dx_t;
      Eigen::Matrix<Scalar,N,1> dx_tm1;
      Eigen::Matrix<Scalar,N,1> dx_tm2;
      double                    dtm1;
      bool                      initialized;

    public:

      inline void integrate(const double dt);

      inline RnSimpsonIntegrator();

      inline ~RnSimpsonIntegrator();

      template<class X1, class DX1, class X2, class DX2>
      inline void initialize(const Eigen::MatrixBase<X1>& x_tm1, 
        const Eigen::MatrixBase<DX1>& dx_tm1,
        const Eigen::MatrixBase<X2>& x_tm2, 
        const Eigen::MatrixBase<DX2>& dx_tm2,
        const double dtm1);

      inline bool isInitialized() const;

      inline void cleanup();

      template<class X>
      inline void set_dx_t(const Eigen::MatrixBase<X>& dx_t );

      inline Eigen::Matrix<Scalar,N,1> get_x_t() const;

    };



    template<typename Scalar, int N>
    RnSimpsonIntegrator<Scalar,N>::RnSimpsonIntegrator () :
    initialized(false)
    {
      ;
    }

    template<typename Scalar, int N>
    RnSimpsonIntegrator<Scalar,N>::~RnSimpsonIntegrator ()
    {
      ;
    }

    template<typename Scalar, int N>
    void RnSimpsonIntegrator<Scalar,N>::integrate (const double dt)
    {
      // si dt == dtm1 :
      //x_t = x_tm2 + ( dx_tm2 + dx_tm1*4 + dx_t ) * dt / 3;

      // On approxime dx(t) par g(t) un polynome d'ordre 2 tq 
      // # g( 0 ) = dx_tm2
      // # g( dtm1 ) = dx_tm1
      // # g( dtm1 + dt ) = dx_t

      // on cherche a,b et c tq g(t) = a t*t + b t + c
      // de façon triviale : c = dx_tm2

      // Variable local pour simplifier les ecritures
      const double tau1   = dtm1;
      const double tau2   = dtm1 + dt;
      const double tau1_2 = tau1 * tau1;
      const double tau2_2 = tau2 * tau2;
      const double tau2_3 = tau2_2 * tau2;
      const Eigen::Matrix<Scalar,N,N> id = Eigen::Matrix<Scalar,N,N>::Identity();

      // on détermine les coeff du polynome
      Eigen::Matrix<Scalar,2*N,2*N> mat;
      mat.topLeftCorner<N,N>()      = id * tau1_2;
      mat.topRightCorner<N,N>()     = id * tau1;
      mat.bottomLeftCorner<N,N>()   = id * tau2_2;
      mat.bottomRightCorner<N,N>()  = id * tau2;
      Eigen::Matrix<Scalar,2*N,1> vec;
      vec.topLeftCorner<N,1>()    = dx_tm1 - dx_tm2;
      vec.bottomLeftCorner<N,1>() =  dx_t  - dx_tm2;
      const Eigen::Matrix<Scalar,2*N,1> ab = mat.inverse() * vec;
      const Eigen::Matrix<Scalar,N,1> a = ab.topLeftCorner<N,1>();
      const Eigen::Matrix<Scalar,N,1> b = ab.bottomLeftCorner<N,1>();
      const Eigen::Matrix<Scalar,N,1> c = dx_tm2;
      
      // on calcul l'intégrale algébriquement
      const Eigen::Matrix<Scalar,N,1> I = a * tau2_3 / 3.0 + b * tau2_2 / 2.0 + tau2 * c;

      // on accumule
      x_t = x_tm2 + I;

      // gestion du buffer
      dx_tm2 = dx_tm1;
      dx_tm1 = dx_t;
      x_tm2  = x_tm1;
      x_tm1  = x_t;
      dtm1   = dt;
    }

    template<typename Scalar, int N>
    template<class X1, class DX1, class X2, class DX2>
    void RnSimpsonIntegrator<Scalar,N>::initialize(const Eigen::MatrixBase<X1>&  _x_tm1, 
      const Eigen::MatrixBase<DX1>& _dx_tm1,
      const Eigen::MatrixBase<X2>&  _x_tm2, 
      const Eigen::MatrixBase<DX2>& _dx_tm2,
      const double _dtm1)
    {
      //EIGEN_STATIC_ASSERT((Eigen::ei_is_same_type<Scalar, typename Eigen::ei_traits<X1>::Scalar>::ret),
      //  YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)
      //EIGEN_STATIC_ASSERT((Eigen::ei_is_same_type<Scalar, typename Eigen::ei_traits<DX1>::Scalar>::ret),
      //  YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)
      //EIGEN_STATIC_ASSERT((Eigen::ei_is_same_type<Scalar, typename Eigen::ei_traits<X2>::Scalar>::ret),
      //  YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)
      //EIGEN_STATIC_ASSERT((Eigen::ei_is_same_type<Scalar, typename Eigen::ei_traits<DX2>::Scalar>::ret),
      //  YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)

      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(X1,  N, 1)
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DX1, N, 1)
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(X2,  N, 1)
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DX2, N, 1)

      x_tm1  = _x_tm1;
      dx_tm1 = _dx_tm1;
      x_tm2  = _x_tm2;
      dx_tm2 = _dx_tm2;
      dtm1   = _dtm1;
      initialized = true;
    }

    template<typename Scalar, int N>
    bool RnSimpsonIntegrator<Scalar,N>::isInitialized () const
    {
      return initialized;
    }

    template<typename Scalar, int N>
    void RnSimpsonIntegrator<Scalar,N>::cleanup()
    {
      initialized = false;
    }

    template<typename Scalar, int N>
    template<class X>
    void RnSimpsonIntegrator<Scalar,N>::set_dx_t(const Eigen::MatrixBase<X>& _dx_t )
    {
      //EIGEN_STATIC_ASSERT((Eigen::ei_is_same_type<Scalar, typename Eigen::ei_traits<X>::Scalar>::ret),
      //  YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)

        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(X, N, 1)

      dx_t   = _dx_t;
    }

    template<typename Scalar, int N>
    Eigen::Matrix<Scalar,N,1> RnSimpsonIntegrator<Scalar,N>::get_x_t() const
    {
      return this->x_t;
    }

  }
}
