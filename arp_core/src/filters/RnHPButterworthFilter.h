#pragma once

#include "lgsm/lgsm"

namespace xde
{
  namespace math
  {
    template<typename Scalar, int N>
    class RnHPButterworthFilter
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
      Eigen::Matrix<Scalar,N,1> x_raw;
      Eigen::Matrix<Scalar,N,1> x_filtered;
      
      /* internal buffer */
      Eigen::Matrix<Scalar,N,1> U[2];
      Eigen::Matrix<Scalar,N,1> Y[2];
      
      
    public:
      inline RnHPButterworthFilter (const double samplingPeriod, const double cutoffFrequency);
      inline ~RnHPButterworthFilter ();

      inline void setParameters (const double samplingPeriod, const double cutoffFrequency);
      inline double getSamplingPeriod () const;
      inline double getCutOffFrequency () const;
      
      template<class X>
      inline void initialize (const Eigen::MatrixBase<X>& Y0);
      inline bool isInitialized () const;
      inline void reset ();
      
      template<class X>
      inline void set_x_raw(const Eigen::MatrixBase<X>& Uk);
      inline Eigen::Matrix<Scalar,N,1> get_x_filtered();
      
      inline void compute();

    private:
      inline void updateCacheValues ();
};



template<typename Scalar, int N>
RnHPButterworthFilter<Scalar,N>::RnHPButterworthFilter (const double samplingPeriod, const double cutoffFrequency)
{
  /* parameters and cache values */
  this->samplingPeriod = samplingPeriod;
  this->cutoffFrequency = cutoffFrequency;

  cacheValues_areUpToDate = false;
  initDone = false;
  
  U[0] = Eigen::Matrix<Scalar,N,1>::Zero();
  U[1] = Eigen::Matrix<Scalar,N,1>::Zero();
  Y[0] = Eigen::Matrix<Scalar,N,1>::Zero();
  Y[1] = Eigen::Matrix<Scalar,N,1>::Zero();
  
  x_raw      = Eigen::Matrix<Scalar,N,1>::Zero();
  x_filtered = Eigen::Matrix<Scalar,N,1>::Zero();
}

template<typename Scalar, int N>
RnHPButterworthFilter<Scalar,N>::~RnHPButterworthFilter ()
{
  ;
}

template<typename Scalar, int N>
void RnHPButterworthFilter<Scalar,N>::setParameters (const double samplingPeriod, const double cutoffFrequency)
{
  if ((samplingPeriod != this->samplingPeriod) || (cutoffFrequency != this->cutoffFrequency)) {

    this->samplingPeriod = samplingPeriod;
    this->cutoffFrequency = cutoffFrequency;

    cacheValues_areUpToDate = false;
  }
}

template<typename Scalar, int N>
double RnHPButterworthFilter<Scalar,N>::getSamplingPeriod () const
{
  return this->samplingPeriod;
}

template<typename Scalar, int N>
double RnHPButterworthFilter<Scalar,N>::getCutOffFrequency () const
{
  return this->cutoffFrequency;
}

template<typename Scalar, int N>
template<class X>
void RnHPButterworthFilter<Scalar,N>::initialize (const Eigen::MatrixBase<X>& Y0)
{
  EIGEN_STATIC_ASSERT((Eigen::ei_is_same_type<Scalar, typename Eigen::ei_traits<X>::Scalar>::ret),
    YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)

  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(X,  N, 1)

  U[0] = Y0;
  U[1] = Y0;

  Y[0] = Y0;
  Y[1] = Y0;

  initDone = true;
}

template<typename Scalar, int N>
bool RnHPButterworthFilter<Scalar,N>::isInitialized () const
{
  return initDone;
}

template<typename Scalar, int N>
void RnHPButterworthFilter<Scalar,N>::reset ()
{
  initDone = false;
  cacheValues_areUpToDate = false;
}


template<typename Scalar, int N>
template<class X>
void RnHPButterworthFilter<Scalar,N>::set_x_raw(const Eigen::MatrixBase<X>& Uk)
{
  EIGEN_STATIC_ASSERT((Eigen::ei_is_same_type<Scalar, typename Eigen::ei_traits<X>::Scalar>::ret),
    YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)

  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(X,  N, 1)
  
  x_raw = Uk;
}

template<typename Scalar, int N>
Eigen::Matrix<Scalar,N,1> RnHPButterworthFilter<Scalar,N>::get_x_filtered()
{
  return x_filtered;
}


template<typename Scalar, int N>
void RnHPButterworthFilter<Scalar,N>::compute()
{
  updateCacheValues ();
  
  x_filtered = ( x_raw * a[0]
               + U[0]  * a[1]
               + U[1]  * a[2]
               - Y[0]  * b[1]
               - Y[1]  * b[2] ) / b[0];
               
  // Shift filter state
  U[1] = U[0];
  U[0] = x_raw;
  Y[1] = Y[0];
  Y[0] = x_filtered;
}


template<typename Scalar, int N>
void RnHPButterworthFilter<Scalar,N>::updateCacheValues ()
{
  if (!cacheValues_areUpToDate) 
  {
    samplingFrequency = 1.0 / samplingPeriod;

    omega_A = tan (M_PI/2 - M_PI * cutoffFrequency * samplingPeriod);

    a[0] = omega_A * omega_A;
    a[1] = - 2.0 * a[0];
    a[2] = a[0];

    b[0] = 1.0 + a[0] + sqrt (2.0) * omega_A;
    b[1] = - 2.0 * ( a[0] - 1 );
    b[2] = 1.0 + a[0] - sqrt (2.0) * omega_A;

    cacheValues_areUpToDate = true;
  }
}



}
}
