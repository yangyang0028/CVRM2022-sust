#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

class KalmanFilter {
private:
    int stateSize; //state variable's dimenssion
    int measSize; //measurement variable's dimession
    int uSize; //control variables's dimenssion
    Eigen::VectorXd x;
    Eigen::VectorXd z;
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::VectorXd u;
    Eigen::MatrixXd P;//coveriance
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;//measurement noise covariance
    Eigen::MatrixXd Q;//process noise covariance
public:
    KalmanFilter(int stateSize_, int measSize_,int uSize_);
    ~KalmanFilter(){}
    void init(Eigen::VectorXd &x_, Eigen::MatrixXd& P_,Eigen::MatrixXd& R_, Eigen::MatrixXd& Q_);
    Eigen::VectorXd predict(Eigen::MatrixXd& A_);
    Eigen::VectorXd predict(Eigen::MatrixXd& A_, Eigen::MatrixXd &B_, Eigen::VectorXd &u_);
    Eigen::VectorXd update(Eigen::MatrixXd& H_,Eigen::VectorXd z_meas);
};

#endif