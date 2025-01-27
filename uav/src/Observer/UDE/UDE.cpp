#include "UDE.h"

namespace Observer {

UDE::UDE() { 
    firstIteration_trans = true;
    firstIteration_rot = true;
    B_pinv_trans = B_trans.completeOrthogonalDecomposition().pseudoInverse();
    B_pinv_rot = B_rot.completeOrthogonalDecomposition().pseudoInverse(); 
    Omega_UDE_trans = 60.0f * Eigen::Matrix3f::Identity();
    Omega_UDE_rot = 80.0f * Eigen::Matrix3f::Identity();
    xi_UDE_trans = Eigen::Vector3f::Zero();
    xi_UDE_rot = Eigen::Vector3f::Zero();
    x_trans = Eigen::VectorXf::Zero(6);
    x_rot = Eigen::VectorXf::Zero(6);
    dx_trans = Eigen::VectorXf::Zero(6);
    dx_rot = Eigen::VectorXf::Zero(6);
    #ifdef PARAMSIM_H
        debugFilePath_trans = SAVE_UDE_TRANS_DEBUG_FILE_PATH_CSV;
        debugOutputFile_trans.open(debugFilePath_trans, std::ios::trunc);
        if (debugOutputFile_trans.is_open()) {
            debugOutputFile_trans << "dt,xi_x,xi_y,xi_z,dot_xi_x,dot_xi_y,dot_xi_z\n";
        }
        debugFilePath_rot = SAVE_UDE_ROT_DEBUG_FILE_PATH_CSV;
        debugOutputFile_rot.open(debugFilePath_rot, std::ios::trunc);
        if (debugOutputFile_rot.is_open()) {
            debugOutputFile_rot << "dt,xi_x,xi_y,xi_z,dot_xi_x,dot_xi_y,dot_xi_z\n";
        }
    #endif
    initialize(); 
}

UDE::~UDE() {}

void UDE::resetUDE(){
    firstIteration_trans = true;
    firstIteration_rot = true;
}

flair::core::Vector3Df UDE::EstimateDisturbance_trans(const flair::core::Vector3Df& p_aux, const flair::core::Vector3Df& dp_aux, float dt) {
    Eigen::Vector3f p(p_aux.x, p_aux.y, p_aux.z);
    Eigen::Vector3f dp(dp_aux.x, dp_aux.y, dp_aux.z);
    
    Eigen::VectorXf x_t(6);
    x_t << p, dp;
    if (firstIteration_trans)
    {
        xi_UDE_trans = Omega_UDE_trans * (B_pinv_trans * x_t);
        firstIteration_trans = false;
    }
    Eigen::Vector3f xi_dot_trans;
    xi_dot_trans = -Omega_UDE_trans * xi_UDE_trans
                 - (Omega_UDE_trans * Omega_UDE_trans * (B_pinv_trans * x_t))
                 + Omega_UDE_trans * (B_pinv_trans * (A_trans * x_t))
                 - Omega_UDE_trans * (u_thrust - Eigen::Vector3f(0, 0, g * mass));
    xi_UDE_trans += dt * xi_dot_trans;
    w_hat_trans = xi_UDE_trans + Omega_UDE_trans * (B_pinv_trans * x_t);
    dx_trans = A_trans * x_t + B_trans * (u_thrust - Eigen::Vector3f(0, 0, g * mass)) + B_trans * w_hat_trans;
    x_trans = x_t;
    #ifdef SAVE_STATE_ESTIMATION_CSV
        SaveStateEstimationCSV(x_trans, dx_trans, w_hat_trans, "TranslationalEstimation.csv");
    #endif
    #ifdef SAVE_UDE_DEBUG_CSV
        SaveUDEDebugCSV(u_thrust, xi_dot_trans, dt, "trans");
    #endif
    return flair::core::Vector3Df(w_hat_trans.x(),w_hat_trans.y(),w_hat_trans.z());
}

flair::core::Vector3Df UDE::EstimateDisturbance_rot(const flair::core::Quaternion& q_aux, const flair::core::Vector3Df& omega_aux, float dt) {
    Eigen::Quaternionf q(q_aux.q0, q_aux.q1, q_aux.q2, q_aux.q3);
    Eigen::Vector3f omega(omega_aux.x, omega_aux.y, omega_aux.z);
    
    Eigen::Vector3f u_torque_UDE = u_torque - omega.cross(J * omega);
    x_rot = Eigen::VectorXf(6);
    // x_rot << q.vec(), omega;
    Eigen::Vector3f q_rvec = rotvec(q);
    x_rot << q_rvec, omega;
    if (firstIteration_rot)
    {
        xi_UDE_rot = -Omega_UDE_rot * (B_pinv_rot * x_rot);
        firstIteration_rot = false;
    }
    Eigen::Vector3f xi_dot_rot;
    xi_dot_rot = -Omega_UDE_rot * xi_UDE_rot
                 - (Omega_UDE_rot * Omega_UDE_rot * (B_pinv_rot * x_rot))
                 + Omega_UDE_rot * (B_pinv_rot * (A_rot * x_rot))
                 - Omega_UDE_rot * (u_torque_UDE);
    xi_UDE_rot += dt * xi_dot_rot;
    w_hat_rot = xi_UDE_rot + Omega_UDE_rot * (B_pinv_rot * x_rot);
    dx_rot = A_rot * x_rot + B_rot * u_torque_UDE + B_rot * w_hat_rot;
    x_rot += dt * dx_rot;
    #ifdef SAVE_STATE_ESTIMATION_CSV
        SaveStateEstimationCSV(x_rot, dx_rot, w_hat_rot, "RotationalEstimation.csv");
    #endif
    #ifdef SAVE_UDE_DEBUG_CSV
        SaveUDEDebugCSV(xi_UDE_rot, xi_dot_rot, dt, "rot");
    #endif

    return flair::core::Vector3Df(w_hat_rot.x(), w_hat_rot.y(), w_hat_rot.z());
}

void UDE::SetNewGainUDE_trans(Eigen::Matrix3f &Omega_trans, Eigen::Vector3f &p, Eigen::Vector3f &dp) {
    Eigen::VectorXf x_trans(6);
    x_trans << p, dp;
    Omega_UDE_trans = Omega_trans;
    if (xi_UDE_trans.size() != 3) {
        xi_UDE_trans = Eigen::Vector3f::Zero();
    }
    xi_UDE_trans = -Omega_UDE_trans * (B_pinv_trans * x_trans);
}

void UDE::SaveUDEDebugCSV(const Eigen::Vector3f& xi, const Eigen::Vector3f& dot_xi, const float& dt, const std::string& dynamic){
    std::ofstream& file = (dynamic == "trans") ? debugOutputFile_trans : debugOutputFile_rot;
    if (file.is_open()) {
        file << dt << ",";
        for (int i = 0; i < xi.size(); ++i) {
            file << xi(i) << ",";
        }
        for (int i = 0; i < dot_xi.size(); ++i) {
            file << dot_xi(i);
            if (i < dot_xi.size() - 1) file << ",";
        }
        file << "\n";
    }
}

} // namespace Observer
