#include "SuperTwist.h"

namespace Observer {

SuperTwist::SuperTwist() {
    L_trans = 3.0f;
    L_rot = 2.5f;

    beta0_trans = 1.5f;
    beta1_trans = 1.1f;

    beta0_rot = 1.5f;
    beta1_rot = 1.1f;

    lambda0_trans = beta0_trans * std::sqrt(L_trans);
    lambda1_trans = beta1_trans * L_trans;
    lambda0_rot = beta0_rot * std::sqrt(L_rot);
    lambda1_rot = beta1_rot * L_rot;

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


flair::core::Vector3Df SuperTwist::EstimateDisturbance_trans(const flair::core::Vector3Df& p_aux, const flair::core::Vector3Df& dp_aux, const flair::core::Vector3Df& u_thrust_aux, float dt) {
    Eigen::Vector3f p(p_aux.x, p_aux.y, p_aux.z);
    Eigen::Vector3f dp(dp_aux.x, dp_aux.y, dp_aux.z);
    Eigen::Vector3f u_thrust(u_thrust_aux.x, u_thrust_aux.y, u_thrust_aux.z);

    
    Eigen::Vector3f e_p = p - Eigen::Vector3f(x_trans[0], x_trans[1], x_trans[2]);
    Eigen::Vector3f e_dp = dp - Eigen::Vector3f(x_trans[3], x_trans[4], x_trans[5]);
    Eigen::Vector3f s = c_p_trans * e_p + c_dp_trans * e_dp;
    Eigen::Vector3f omega = lambda0_trans * s.cwiseAbs().cwiseSqrt().cwiseProduct(s.cwiseSign()) + eta_trans;
    Eigen::Vector3f deta = lambda1_trans * s.cwiseSign();
    eta_trans += dt * deta;
    Eigen::VectorXf dx_hat = A_trans * x_trans + B_trans * (u_thrust - Eigen::Vector3f(0, 0, g * mass/10)) + B_trans * omega;
    x_trans += dt * dx_hat;
    w_hat_trans = omega;
    //SaveStateEstimationCSV(x_trans, dx_hat, w_hat_trans, "TranslationalEstimation.csv");
    return flair::core::Vector3Df(w_hat_trans.x(),w_hat_trans.y(),w_hat_trans.z());
}

flair::core::Vector3Df SuperTwist::EstimateDisturbance_rot(const flair::core::Quaternion& q_aux, const flair::core::Vector3Df& omega_aux, const flair::core::Vector3Df& u_torque_aux, float dt) {
    Eigen::Quaternionf q(q_aux.q0, q_aux.q1, q_aux.q2, q_aux.q3);
    Eigen::Vector3f omega(omega_aux.x, omega_aux.y, omega_aux.z);
    Eigen::Vector3f u_torque(u_torque_aux.x, u_torque_aux.y, u_torque_aux.z);
    
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
    //SaveStateEstimationCSV(x_rot, dx_hat, w_hat_rot, "RotationalEstimation.csv");
    return flair::core::Vector3Df(w_hat_rot.x(), w_hat_rot.y(), w_hat_rot.z());
}

} // namespace Observer
