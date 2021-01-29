
#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

#include <string>
#include "Eigen/Dense"

namespace msf {
namespace common {

  
  template <typename T, unsigned int N>
  Eigen::Matrix<T, N, N> PseudoInverse(const Eigen::Matrix<T, N, N> &m,
                                      const double epsilon = 1.0e-6) {
    Eigen::JacobiSVD<Eigen::Matrix<T, N, N>> svd(
        m, Eigen::ComputeFullU | Eigen::ComputeFullV);
    return svd.matrixV() *
          (svd.singularValues().array().abs() > epsilon)
              .select(svd.singularValues().array().inverse(), 0)
              .matrix()
              .asDiagonal() *
          svd.matrixU().adjoint();
  }

  template <typename T, unsigned int M, unsigned int N>
  Eigen::Matrix<T, N, M> PseudoInverse(const Eigen::Matrix<T, M, N>& m,
          const double epsilon = 1.0e-6) {
      Eigen::Matrix<T, M, M> t = m * m.transpose();
      return m.transpose() * PseudoInverse<T, M>(t);
  }

  template <typename T, unsigned int N>
  Eigen::Matrix<T, N, N> ContinuousToDiscrete(const Eigen::Matrix<T, N, N> &m_c,
                                              const double ts) {
    Eigen::Matrix<T, N, N> m_identity = Eigen::Matrix<T, N, N>::Identity();
    Eigen::Matrix<T, N, N> m_d = (m_identity + ts * 0.5 * m_c) *
                                PseudoInverse<T, N>(m_identity - ts * 0.5 * m_c);
    return m_d;
  }

/**
 * @class KalmanFilter
 *
 * @brief Implements a discrete-time Kalman filter.
 *
 * @param XN dimension of state
 * @param ZN dimension of observations
 * @param UN dimension of controls
 */
template <typename T, unsigned int XN, unsigned int ZN, unsigned int UN>
class KalmanFilter {
 public:
  /**
   * @brief Constructor which defers initialization until the initial state
   * distribution parameters are set (with SetStateEstimate),
   * typically on the first observation
   */
  KalmanFilter() {
    F_.setIdentity();
    Q_.setZero();
    H_.setIdentity();
    R_.setZero();
    B_.setZero();
    IKH_.setZero();
  }

  /**
   * @brief Sets the initial state belief distribution.
   *
   * @param x Mean of the state belief distribution
   * @param P Covariance of the state belief distribution
   */
  void SetStateEstimate(const Eigen::Matrix<T, XN, 1> &x,
                        const Eigen::Matrix<T, XN, XN> &P) {
    x_ = x;
    P_ = P;
    is_initialized_ = true;
  }

  /**
   * @brief Constructor which fully initializes the Kalman filter
   * @param x Mean of the state belief distribution
   * @param P Covariance of the state belief distribution
   */
  KalmanFilter(const Eigen::Matrix<T, XN, 1> &x,
               const Eigen::Matrix<T, XN, XN> &P)
      : KalmanFilter() {
    SetStateEstimate(x, P);
  }

  /**
   * @brief Destructor
   */
  virtual ~KalmanFilter() {}

  /**
   * @brief Changes the system transition function under zero control.
   *
   * @param F New transition matrix
   */
  void SetTransitionMatrix(const Eigen::Matrix<T, XN, XN> &F) { F_ = F; }

  /**
   * @brief Changes the covariance matrix of the transition noise.
   *
   * @param Q New covariance matrix
   */
  void SetTransitionNoise(const Eigen::Matrix<T, XN, XN> &Q) { Q_ = Q; }

  /**
   * @brief Changes the observation matrix, which maps states to observations.
   *
   * @param H New observation matrix
   */
  void SetObservationMatrix(const Eigen::Matrix<T, ZN, XN> &H) { H_ = H; }

  /**
   * @brief Changes the covariance matrix of the observation noise.
   *
   * @param R New covariance matrix
   */
  void SetObservationNoise(const Eigen::Matrix<T, ZN, ZN> &R) { R_ = R; }

  /**
   * @brief Changes the covariance matrix of current state belief distribution.
   *
   * @param P New state covariance matrix
   */
  void SetStateCovariance(const Eigen::Matrix<T, XN, XN> &P) { P_ = P; }

  /**
   * @brief Changes the control matrix in the state transition rule.
   *
   * @param B New control matrix
   */
  void SetControlMatrix(const Eigen::Matrix<T, XN, UN> &B) { B_ = B; }

  /**
   * @brief Get the system transition function under zero control.
   *
   * @return Transition matrix.
   */
  const Eigen::Matrix<T, XN, XN> &GetTransitionMatrix() const { return F_; }

  /**
   * @brief Get the covariance matrix of the transition noise.
   *
   * @return Covariance matrix
   */
  const Eigen::Matrix<T, XN, XN> &GetTransitionNoise() const { return Q_; }

  /**
   * @brief Get the observation matrix, which maps states to observations.
   *
   * @return Observation matrix
   */
  const Eigen::Matrix<T, ZN, XN> &GetObservationMatrix() const { return H_; }

  /**
   * @brief Get the covariance matrix of the observation noise.
   *
   * @return Covariance matrix
   */
  const Eigen::Matrix<T, ZN, ZN> &GetObservationNoise() const { return R_; }

  /**
   * @brief Get the control matrix in the state transition rule.
   *
   * @return Control matrix
   */
  const Eigen::Matrix<T, XN, UN> &GetControlMatrix() const { return B_; }

  /**
   * @brief Updates the state belief distribution given the control input u.
   *
   * @param u Control input (by default, zero)
   */
  void Predict(
      const Eigen::Matrix<T, UN, 1> &u = Eigen::Matrix<T, UN, 1>::Zero());

  /**
   * @brief Updates the state belief distribution given an observation z.
   *
   * @param z Observation
   */
  T Correct(const Eigen::Matrix<T, ZN, 1> &z, const double limit);

    /**
   * @brief Updates the state belief distribution given an observation z.
   *
   * @param z Observation
   */
  T Correct2(const Eigen::Matrix<T, ZN, 1> &z, const double limit);

  /**
   * @brief Changes the control matrix in the state transition rule.
   *
   * @param B New control matrix
   */
  void ResetStateArray(unsigned int startIndex, unsigned int endIndex);

  /**
   * @brief Gets mean of our current state belief distribution
   *
   * @return State vector
   */
  Eigen::Matrix<T, XN, 1> GetStateEstimate() const { return x_; }

  /**
   * @brief Gets covariance of our current state belief distribution
   *
   * @return Covariance matrix
   */
  Eigen::Matrix<T, XN, XN> GetStateCovariance() const { return P_; }


 private:
  // Mean of current state belief distribution
  Eigen::Matrix<T, XN, 1> x_;

  // Covariance of current state belief dist
  Eigen::Matrix<T, XN, XN> P_;

  // State transition matrix under zero control
  Eigen::Matrix<T, XN, XN> F_;

  // Covariance of the state transition noise
  Eigen::Matrix<T, XN, XN> Q_;

  // Observation matrix
  Eigen::Matrix<T, ZN, XN> H_;

  // Covariance of observation noise
  Eigen::Matrix<T, ZN, ZN> R_;

  // Control matrix in state transition rule
  Eigen::Matrix<T, XN, UN> B_;

  // Innovation; marked as member to prevent memory re-allocation.
  Eigen::Matrix<T, ZN, 1> y_;

  // Innovation covariance; marked as member to prevent memory re-allocation.
  Eigen::Matrix<T, ZN, ZN> S_;

  // Kalman gain; marked as member to prevent memory re-allocation.
  Eigen::Matrix<T, XN, ZN> K_;

  Eigen::Matrix<T, XN, XN> IKH_;

  // true iff SetStateEstimate has been called.
  bool is_initialized_ = false;
};

template <typename T, unsigned int XN, unsigned int ZN, unsigned int UN>
inline void KalmanFilter<T, XN, ZN, UN>::Predict(
    const Eigen::Matrix<T, UN, 1> &u) {
  if(is_initialized_)
  {
    x_ = F_ * x_ + B_ * u;
    P_ = F_ * P_ * F_.transpose() + Q_;
  }
}

template <typename T, unsigned int XN, unsigned int ZN, unsigned int UN>                                                                                                                                                                                                                                                                                                                    
inline T KalmanFilter<T, XN, ZN, UN>::Correct(
    const Eigen::Matrix<T, ZN, 1> &z,const double limit) {
  if(is_initialized_)
  {
    Eigen::Matrix<T,1,1> lambda;

    y_ = z - H_ * x_;

    S_ = H_ * P_ * H_.transpose() + R_;

    K_ = P_ * H_.transpose() * (PseudoInverse<T, ZN>(S_));

    lambda = y_.transpose() * (PseudoInverse<T, ZN>(S_)) * y_;

    for(int k = 18; k < XN; k++) K_(k) = 0.0;

    if(lambda(0) <= limit)
    {
      x_ = x_ + K_ * y_;

      IKH_ = (Eigen::Matrix<T, XN, XN>::Identity() - K_ * H_);

      P_ = IKH_ * P_ * IKH_.transpose() + K_ * R_ * K_.transpose();

    }

    return lambda(0);
  }

  return -1;
}

template <typename T, unsigned int XN, unsigned int ZN, unsigned int UN>                                                                                                                                                                                                                                                                                                                    
inline T KalmanFilter<T, XN, ZN, UN>::Correct2(
    const Eigen::Matrix<T, ZN, 1> &z,const double limit) {
  if(is_initialized_)
  {
    Eigen::Matrix<T,1,1> lambda;

    y_ = z - H_ * x_;

    S_ = H_ * P_ * H_.transpose() + R_;

    K_ = P_ * H_.transpose() * (PseudoInverse<T, ZN>(S_));

    lambda = y_.transpose() * (PseudoInverse<T, ZN>(S_)) * y_;

    for(int k = 0; k < 18; k++) K_(k) = 0.0;

    if(lambda(0) <= limit)
    {
      x_ = x_ + K_ * y_;

      IKH_ = (Eigen::Matrix<T, XN, XN>::Identity() - K_ * H_);

      P_ = IKH_ * P_ * IKH_.transpose() + K_ * R_ * K_.transpose();

    }

    return lambda(0);
  }

  return -1;
}

template <typename T, unsigned int XN, unsigned int ZN, unsigned int UN>
inline void KalmanFilter<T, XN, ZN, UN>::ResetStateArray(
    unsigned int startIndex, unsigned int endIndex) {
      for(unsigned int k = startIndex; k <= endIndex ; k++){
        x_(k) = 0.0;
      }
    }

}  // namespace localization
}  // namespace msf

#endif /* _KALMAN_FILTER_H_ */
