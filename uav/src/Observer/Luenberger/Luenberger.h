#ifndef LUENBERGER_H
#define LUENBERGER_H

#include "../ObserverBase.h"
#include <Eigen/Dense>
#include <string>
#include <fstream>
#include <iostream>

namespace Observer {

class Luenberger : public ObserverBase {
public:

    bool firstIteration_trans;
    bool firstIteration_rot;

    Eigen::MatrixXf A_ext_trans;
    Eigen::MatrixXf A_ext_rot;

    Eigen::MatrixXf B_ext_trans;
    Eigen::MatrixXf B_ext_rot;

    Eigen::MatrixXf C_ext_trans;
    Eigen::MatrixXf C_ext_rot;

    // Observer Gains
    Eigen::MatrixXf L_trans; 
    Eigen::MatrixXf L_rot;   

    // State Estimation
    Eigen::VectorXf x_trans; // Estimated translational state
    Eigen::VectorXf x_rot;   // Estimated rotational state

    // Observer Functions
    flair::core::Vector3Df EstimateDisturbance_trans(const flair::core::Vector3Df& p, const flair::core::Vector3Df& dp, const flair::core::Vector3Df& u_thrust, float dt) override;
    flair::core::Vector3Df EstimateDisturbance_rot(const flair::core::Quaternion& q, const flair::core::Vector3Df& omega, const flair::core::Vector3Df& u_torque, float dt) override;
    
    Luenberger();
    ~Luenberger();

    // Set Gain Functions
    void SetNewGainL_trans(const Eigen::MatrixXf& L_trans_new);
    void SetNewGainL_rot(const Eigen::MatrixXf& L_rot_new);

private:
    void InitializeEstimates();
};

} // namespace Observer

#endif // LUENBERGER_H
