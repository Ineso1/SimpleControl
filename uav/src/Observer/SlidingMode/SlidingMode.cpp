#include "SlidingMode.h"
#include <iostream>

namespace Observer {

SlidingMode::SlidingMode() {
    L_trans = 1.0f;
    L_rot = 1.0f;

    epsilon_trans = 0.0001f;
    epsilon_rot = 0.0001f;

    tau_trans = 0.9f;
    tau_rot = 0.9f;

    rho_trans = L_trans + epsilon_trans;
    rho_rot = L_rot + epsilon_rot;

    c_p_trans = 0.1f;
    c_dp_trans = 0.12f;
    c_q_rot = 0.1f;
    c_omega_rot = 0.1f;

    d_est_filtered_trans = Eigen::Vector3f::Zero();
    d_est_filtered_rot = Eigen::Vector3f::Zero();

    x_trans = Eigen::VectorXf::Zero(6);
    x_rot = Eigen::VectorXf::Zero(6);
}

SlidingMode::~SlidingMode() {}

void SlidingMode::SetUpperBounds(float L_trans_new, float L_rot_new) {
    L_trans = L_trans_new;
    L_rot = L_rot_new;
    rho_trans = L_trans + epsilon_trans;
    rho_rot = L_rot + epsilon_rot;
}

void SlidingMode::SetFilterGains(float tau_trans_new, float tau_rot_new) {
    tau_trans = tau_trans_new;
    tau_rot = tau_rot_new;
}

void SlidingMode::SetSlidingGains(float c_p_trans_new, float c_dp_trans_new, float c_q_rot_new, float c_omega_rot_new) {
    c_p_trans = c_p_trans_new;
    c_dp_trans = c_dp_trans_new;
    c_q_rot = c_q_rot_new;
    c_omega_rot = c_omega_rot_new;
}

Eigen::Vector3f SlidingMode::EstimateDisturbance_trans(const Eigen::Vector3f& p, const Eigen::Vector3f& dp, float dt) {
    Eigen::Vector3f e_p = p - Eigen::Vector3f(x_trans[0], x_trans[1], x_trans[2]);
    Eigen::Vector3f e_dp = dp - Eigen::Vector3f(x_trans[3], x_trans[4], x_trans[5]);
    Eigen::Vector3f s = c_p_trans * e_p + c_dp_trans * e_dp;
    Eigen::Vector3f omega = rho_trans * s.array().sign();
    Eigen::VectorXf dx_hat = A_trans * x_trans + B_trans * (u_thrust - Eigen::Vector3f(0, 0, g * mass)) + B_trans * omega;
    x_trans += dt * dx_hat;
    d_est_filtered_trans += (1 / tau_trans) * (-d_est_filtered_trans + omega) * dt;
    w_hat_trans = d_est_filtered_trans;
    SaveStateEstimationCSV(x_trans, dx_hat, w_hat_trans, "TranslationalEstimation.csv");
    return w_hat_trans;
}

Eigen::Vector3f SlidingMode::EstimateDisturbance_rot(const Eigen::Quaternionf& q, const Eigen::Vector3f& omega, float dt) {
    Eigen::Vector3f u_torque_SM = u_torque - omega.cross(J * omega);
    Eigen::Vector3f q_vec = rotvec(q);
    Eigen::Vector3f e_q = q_vec - x_rot.segment<3>(0);
    Eigen::Vector3f e_omega = omega - x_rot.segment<3>(3);
    Eigen::Vector3f s = c_q_rot * e_q + c_omega_rot * e_omega;
    Eigen::Vector3f omega_sm = rho_rot * s.array().sign();
    Eigen::VectorXf dx_hat = A_rot * x_rot + B_rot * u_torque_SM + B_rot * omega_sm;
    x_rot += dt * dx_hat;
    d_est_filtered_rot += (1 / tau_rot) * (-d_est_filtered_rot + omega_sm) * dt;
    w_hat_rot = d_est_filtered_rot;
    SaveStateEstimationCSV(x_rot, dx_hat, w_hat_rot, "RotationalEstimation.csv");
    return w_hat_rot;
}

} // namespace Observer
