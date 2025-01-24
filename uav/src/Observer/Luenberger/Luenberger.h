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
    Eigen::Vector3f EstimateDisturbance_trans(const Eigen::Vector3f& p, const Eigen::Vector3f& dp, float dt) override;
    Eigen::Vector3f EstimateDisturbance_rot(const Eigen::Quaternionf& q, const Eigen::Vector3f& omega, float dt) override;
    
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
