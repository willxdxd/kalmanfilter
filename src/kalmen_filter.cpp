#include "kalmen_filter.h"
#include <iostream>

void KalmenFilter::Initialize(  const size_t state_variable_size,
                                const size_t measurement_size,
                                const Eigen::MatrixXd& H,
                                const Eigen::MatrixXd& x,
                                const Eigen::MatrixXd& P,
                                const Eigen::MatrixXd& F,
                                const Eigen::MatrixXd& Q,
                                const Eigen::MatrixXd& R)
{
    state_variable_size_ = state_variable_size;
    measurement_size_ = measurement_size;
    H_ = H;
    x_ = x;
    P_ = P;
    F_ = F;
    Q_ = Q;
    R_ = R;

    return;
}

void KalmenFilter::Estimate(const Eigen::MatrixXd& z) 
{
    Predict();
    Update(z);

    return;
}

Eigen::MatrixXd KalmenFilter::GetStateX() const 
{
    return x_;
}

Eigen::MatrixXd KalmenFilter::GetStateCovariance() const
{
    return P_;
}

void KalmenFilter::SetStateX(const Eigen::MatrixXd& x) {
    x_ = x;

    return;
}

void KalmenFilter::SetStateCovariance(const Eigen::MatrixXd& P) {
    P_ = P;

    return;
}

void KalmenFilter::Predict()
{
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;

    return;
}

void KalmenFilter::Update(const Eigen::MatrixXd& z) 
{
    
    Eigen::MatrixXd y = z - H_ * x_;

    S_ = H_ * P_ * H_.transpose() + R_;
    K_ = P_ * H_.transpose() * S_.inverse();

    x_ = x_ + K_ * y;

    Eigen::MatrixXd I =  Eigen::MatrixXd::Identity(state_variable_size_, state_variable_size_);
    P_ = (I - K_ * H_) * P_ * (I - K_ * H_).transpose() + K_ * R_ * K_.transpose();

    return;
}

void KalmenFilter::SetStateTransitionFunction(const Eigen::MatrixXd& F) {
    F_ = F;

    return;
}

void KalmenFilter::SetStateTransitionCovariance(const Eigen::MatrixXd& Q) {
    Q_ = Q;
    
    return;
}

