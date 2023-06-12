#ifndef KALMEN_FILTER
#define KALMEN_FILTER

#include <eigen3/Eigen/Dense>

/*
This is the filter designed and implemented for SL project.

This module is mainly responsible for filtering moving objects' tracklets on highway(currently), city(future).

The whole system works as follows
    step 1, initialize kf parameters
    step 2, run predict and update
    step 3, get estimates(new prior)

For this project:
    n:      fixed,      needs init
    mea_n:  fixed,      needs init
    H:      fixed,      needs init
    x:      changing,   needs init
    P:      changing,   needs init
    F:      changing,   needs init and input (Target time)
    Q:      changing,   needs init and input (Target time)
    y:      changing,   
    z:      changing,   needs input
    R:      fixed,      needs init input?? check with lidar equipment 
    S:      changing
    K:      changing

    output
    x:      as estimate
    P:      as estimiate


Below is background of kf:

Parameters commonly used in kf:
    n:      size of track variable
    mea_n:  size of measurement variable
    H:      measurement matrix            mea_n by n,  observation element is 1
    x:      state(prior)                  n by 1,  eg, [x, x_dot, y, y_dot]
    P:      state(prior) covariance       n by n cov
    F:      state transition function     n by n,
    Q:      state transition covariance   n by n cov
    y:      residual                      mea_n by 1
    z:      measurement                   mea_n by 1
    R:      measurement_covariance        mea_n by mea_n
    S:      system uncertainty            mea_n by mea_n
    K:      kalmen gain                   n by mea_n

SL high way kalmen filter parameter specification
    n:      6 (xyz, vx, vy, vz)
    mea_n:  3 (xyz from lidar, vx, vy, vz is hidden)
    x:      [x, y, z, vx, vy, vz], predict( x = Fx ), update( x = x + Ky )  
    H:      [1 0 0 0 0 0]
            [0 0 1 0 0 0]
            [0 0 0 0 1 0]
    P:      cov of x, predict(FPF.T + Q), update(state version is (I - KH)P(I - KH).T + KRK.T)
    F:      [1 dt 0  0 0  0]
            [0  1 0  0 0  0]
            [0  0 1 dt 0  0]
            [0  0 0  1 0  0]
            [0  0 0  0 1 dt]
            [0  0 0  0 0  1]
    Q:      cov of F, needs to be designed by noise model
    y:      z - x_predicted
    z:      [x, y, z]
    R:      cov of z, depends on lidar
    S:      update( HPH.T + R ) 
    K:      update( PH.TS^-1 )

*/


class KalmenFilter {

public:
    KalmenFilter() = default;
    ~KalmenFilter() = default;

    /**
     * @brief initialize filter
     * 
     * @param state_variable_size   n
     * @param measurement_size      mea_n
     * @param H 
     * @param x 
     * @param P 
     * @param F 
     * @param Q 
     * @param R 
     * @return ** void 
     */
    void Initialize(const size_t state_variable_size,
                    const size_t measurement_size,
                    const Eigen::MatrixXd& H,
                    const Eigen::MatrixXd& x,
                    const Eigen::MatrixXd& P,
                    const Eigen::MatrixXd& F,
                    const Eigen::MatrixXd& Q,
                    const Eigen::MatrixXd& R);

    /**
     * @brief do filtering based on measurement
     * 
     * @param z
     * @return ** void 
     */
    void Estimate(const Eigen::MatrixXd& z);

    /**
     * @brief Get the x
     * 
     * @return ** Eigen::MatrixXd 
     */
    Eigen::MatrixXd GetStateX() const;    

    /**
     * @brief Get the state covariance
     * 
     * @return ** Eigen::MatrixXd 
     */
    Eigen::MatrixXd GetStateCovariance() const;

    /**
     * @brief Set the State X
     * 
     * @param x 
     * @return ** void 
     */
    void SetStateX(const Eigen::MatrixXd& x);

    /**
     * @brief Set the State Covariance
     * 
     * @param P 
     * @return ** void 
     */
    void SetStateCovariance(const Eigen::MatrixXd& P);

    /**
     * @brief Set the State Transition Function
     * 
     * @param F 
     * @return ** void 
     */
    void SetStateTransitionFunction(const Eigen::MatrixXd& F);

    /**
     * @brief Set the State Transition Covariance 
     * 
     * @param Q 
     * @return ** void 
     */
    void SetStateTransitionCovariance(const Eigen::MatrixXd& Q);

    /**
     * @brief predict in filter round
     * 
     * @return ** void 
     */
    void Predict();

    /**
     * @brief update in filter round
     * 
     * @param z 
     * @return ** void 
     */
    void Update(const Eigen::MatrixXd& z);

private:

    size_t state_variable_size_;
    size_t measurement_size_;
    
    Eigen::MatrixXd H_;
    Eigen::MatrixXd x_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    Eigen::MatrixXd S_;
    Eigen::MatrixXd K_;

    friend class KalmenFilterTester;
};


#endif
