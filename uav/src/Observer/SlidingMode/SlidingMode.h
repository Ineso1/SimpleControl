#ifndef SLIDINGMODE_H
#define SLIDINGMODE_H

#include "../ObserverBase.h"
#include <Eigen/Dense>
#include <string>
#include <iomanip>
#include <fstream>

namespace Observer {

class SlidingMode : public ObserverBase {
public:
    // Parameters for translational and rotational dynamics
    float L_trans;
    float L_rot;

    float rho_trans;
    float rho_rot;

    float epsilon_trans;
    float epsilon_rot;

    float tau_trans;
    float tau_rot;

    float c_p_trans;
    float c_dp_trans;

    float c_q_rot;
    float c_omega_rot;

    Eigen::Vector3f d_est_filtered_trans; // Filtered disturbance estimate (translational)
    Eigen::Vector3f d_est_filtered_rot;   // Filtered disturbance estimate (rotational)

    // State Estimation
    Eigen::VectorXf x_trans; // Estimated translational state
    Eigen::VectorXf x_rot;   // Estimated rotational state

    std::string debugFilePath_trans;
    std::ofstream debugOutputFile_trans;
    std::string debugFilePath_rot;
    std::ofstream debugOutputFile_rot;

    SlidingMode();
    ~SlidingMode();

    // Overriding disturbance estimation functions
    Eigen::Vector3f EstimateDisturbance_trans(const Eigen::Vector3f& p, const Eigen::Vector3f& dp, float dt) override;
    Eigen::Vector3f EstimateDisturbance_rot(const Eigen::Quaternionf& q, const Eigen::Vector3f& omega, float dt) override;

    // Setters for parameters
    void SetUpperBounds(float L_trans_new, float L_rot_new);
    void SetFilterGains(float tau_trans_new, float tau_rot_new);
    void SetSlidingGains(float c_p_trans_new, float c_dp_trans_new, float c_q_rot_new, float c_omega_rot_new);

};

} // namespace Observer

#endif // SLIDINGMODE_H
