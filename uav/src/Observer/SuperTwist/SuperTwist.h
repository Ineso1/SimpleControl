#ifndef SUPERTWIST_H
#define SUPERTWIST_H

#include "../ObserverBase.h"
#include <Eigen/Dense>
#include <string>
#include <fstream>

namespace Observer {

class SuperTwist : public ObserverBase {
public:
    // Parameters for translational and rotational dynamics
    float L_trans;
    float L_rot;

    float lambda0_trans;
    float lambda1_trans;
    float lambda0_rot;
    float lambda1_rot;

    Eigen::Vector3f eta_trans;
    Eigen::Vector3f eta_rot;

    float beta0_trans;
    float beta1_trans;

    float beta0_rot;
    float beta1_rot;

    float c_p_trans;
    float c_dp_trans;
    float c_q_rot;
    float c_omega_rot;

    // State Estimation
    Eigen::VectorXf x_trans; // Estimated translational state
    Eigen::VectorXf x_rot;   // Estimated rotational state

    std::string debugFilePath_trans;
    std::ofstream debugOutputFile_trans;
    std::string debugFilePath_rot;
    std::ofstream debugOutputFile_rot;

    SuperTwist();
    ~SuperTwist();

    flair::core::Vector3Df EstimateDisturbance_trans(const flair::core::Vector3Df& p, const flair::core::Vector3Df& dp, const flair::core::Vector3Df& u_thrust, float dt) override;
    flair::core::Vector3Df EstimateDisturbance_rot(const flair::core::Quaternion& q, const flair::core::Vector3Df& omega, const flair::core::Vector3Df& u_torque, float dt) override;

    void SetUpperBounds(float L_trans_new, float L_rot_new);
    void SetSlidingGains(float c_p_trans_new, float c_dp_trans_new, float c_q_rot_new, float c_omega_rot_new);

};

} // namespace Observer

#endif // SUPERTWIST_H
