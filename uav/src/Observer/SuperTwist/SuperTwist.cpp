#include "SuperTwist.h"

namespace Observer {

SuperTwist::SuperTwist() {
    L_trans = 3.0f;
    L_rot = 2.5f;

    lambda0_trans = 1.5f * std::sqrt(L_trans);
    lambda1_trans = 1.1f * L_trans;
    lambda0_rot = 1.5f * std::sqrt(L_rot);
    lambda1_rot = 1.1f * L_rot;

    eta_trans = Eigen::Vector3f::Zero();
    eta_rot = Eigen::Vector3f::Zero();

    c_p_trans = 1.5f;
    c_dp_trans = 1.5f;
    c_q_rot = 0.2f;
    c_omega_rot = 0.1f;

    x_trans = Eigen::VectorXf::Zero(6);
    x_rot = Eigen::VectorXf::Zero(6);
}

SuperTwist::~SuperTwist() {}


void SuperTwist::SetUpperBounds(float L_trans_new, float L_rot_new) {
    L_trans = L_trans_new;
    L_rot = L_rot_new;
    lambda0_trans = 1.5f * std::sqrt(L_trans);
    lambda1_trans = 1.1f * L_trans;
    lambda0_rot = 1.5f * std::sqrt(L_rot);
    lambda1_rot = 1.1f * L_rot;
}

void SuperTwist::SetSlidingGains(float c_p_trans_new, float c_dp_trans_new, float c_q_rot_new, float c_omega_rot_new) {
    c_p_trans = c_p_trans_new;
    c_dp_trans = c_dp_trans_new;
    c_q_rot = c_q_rot_new;
    c_omega_rot = c_omega_rot_new;
}


Eigen::Vector3f SuperTwist::EstimateDisturbance_trans(const Eigen::Vector3f& p, const Eigen::Vector3f& dp, float dt) {
    Eigen::Vector3f e_p = p - Eigen::Vector3f(x_trans[0], x_trans[1], x_trans[2]);
    Eigen::Vector3f e_dp = dp - Eigen::Vector3f(x_trans[3], x_trans[4], x_trans[5]);
    Eigen::Vector3f s = c_p_trans * e_p + c_dp_trans * e_dp;
    Eigen::Vector3f omega = lambda0_trans * s.cwiseAbs().cwiseSqrt().cwiseProduct(s.cwiseSign()) + eta_trans;
    Eigen::Vector3f deta = lambda1_trans * s.cwiseSign();
    eta_trans += dt * deta;
    Eigen::VectorXf dx_hat = A_trans * x_trans + B_trans * (u_thrust - Eigen::Vector3f(0, 0, g * mass)) + B_trans * omega;
    x_trans += dt * dx_hat;
    w_hat_trans = omega;
    SaveStateEstimationCSV(x_trans, dx_hat, w_hat_trans, "TranslationalEstimation.csv");
    return w_hat_trans;
}

Eigen::Vector3f SuperTwist::EstimateDisturbance_rot(const Eigen::Quaternionf& q, const Eigen::Vector3f& omega, float dt) {
    Eigen::Vector3f u_torque_ST = u_torque - omega.cross(J * omega);
    Eigen::Vector3f q_vec = rotvec(q);
    Eigen::Vector3f e_q = q_vec - x_rot.segment<3>(0);
    Eigen::Vector3f e_omega = omega - x_rot.segment<3>(3);
    Eigen::Vector3f s = c_q_rot * e_q + c_omega_rot * e_omega;
    Eigen::Vector3f omega_st = lambda0_rot * s.cwiseAbs().cwiseSqrt().cwiseProduct(s.cwiseSign()) + eta_rot;
    Eigen::Vector3f deta = lambda1_rot * s.cwiseSign();
    eta_rot += dt * deta;
    Eigen::VectorXf dx_hat = A_rot * x_rot + B_rot * u_torque_ST + B_rot * omega_st;
    x_rot += dt * dx_hat;
    w_hat_rot = omega_st;
    SaveStateEstimationCSV(x_rot, dx_hat, w_hat_rot, "RotationalEstimation.csv");
    return w_hat_rot;
}

} // namespace Observer
