// KFC.h

#ifndef KFC_H
#define KFC_H

#include "../ObserverBase.h"
#include <Eigen/Dense>
#include <string>
#include <iomanip>
#include <fstream>

namespace Observer {

class KFC : public ObserverBase {
public:

    // First value condition
    // bool firstIteration_trans;
    // bool firstIteration_rot;

    // 
    Eigen::MatrixXf Fk;             // State transition matrix (Discrete-time A)
    Eigen::MatrixXf Bk;             // Input matrix (Discrete-time B)
    Eigen::MatrixXf Hk;             // Observation matrix (C)
    Eigen::VectorXf Zk;             // Measurement vector

    // State Space Estimation
    Eigen::VectorXf Xk;             // State estimate vector
    Eigen::MatrixXf Rk;             // Measurement noise covariance
    Eigen::MatrixXf Qk;             // Process noise covariance
    Eigen::MatrixXf Pk;             // State covariance matrix
    Eigen::MatrixXf Sk;             // Innovation covariance
    Eigen::MatrixXf Kk;             // Kalman gain
    
    float dt;                      // Sampling time
    bool firstUpdate;        // Flag for first update

    KFC();
    ~KFC();

    void resetKFC();
    // Kalman filter step
    void KFC_estimate(const Eigen::Vector3f& p, const Eigen::Vector3f& dp, const Eigen::Vector3f& u_thrust);

    // Get state
    // Provide access to the current state of the system estimated by the Kalman filter
    void getState(Eigen::Vector3f& p, Eigen::Vector3f& dp) const;
};

} // namespace Observer

#endif // KFC_H