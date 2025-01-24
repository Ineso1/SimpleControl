#include "StateSpace.h"

namespace Observer {

StateSpace::StateSpace() {
    // Model Parameters
    g = 9.81f;
    mass = 1.0f;
    J = Eigen::Matrix3f::Identity();

    // State Vectors
    p = Eigen::Vector3f::Zero();
    dp = Eigen::Vector3f::Zero();
    ddp = Eigen::Vector3f::Zero();
    u_thrust = Eigen::Vector3f::Zero();

    q = Eigen::Quaternionf::Identity();
    dq = Eigen::Quaternionf::Identity();
    omega = Eigen::Vector3f::Zero();
    domega = Eigen::Vector3f::Zero();
    u_torque = Eigen::Vector3f::Zero();

    // State Space Matrices for Translational Dynamics
    A_trans = Eigen::MatrixXf::Zero(6, 6);
    A_trans.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity();
    B_trans = Eigen::MatrixXf::Zero(6, 3);
    B_trans.block<3, 3>(3, 0) = Eigen::Matrix3f::Identity() / mass;
    C_trans = Eigen::MatrixXf::Zero(3, 6);
    C_trans.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
    // C_trans.block<3, 3>(3, 0) = Eigen::Matrix3f::Identity();
    B_pinv_trans = B_trans.completeOrthogonalDecomposition().pseudoInverse();

    w_hat_trans = Eigen::Vector3f::Zero();
    Omega_UDE_trans = 10.0f * Eigen::Matrix3f::Identity();
    xi_UDE_trans = Eigen::Vector3f::Zero();

    desired_eigenvalues_trans = (Eigen::VectorXf(6) << -10, -10, -10, -15, -15, -15).finished();
    L_trans = (Eigen::MatrixXf(6, 3) << 
                25.0f, 0.0f, 0.0f,
                0.0f, 25.0f, 0.0f,
                0.0f, 0.0f, 25.0f,
                150.0f, 0.0f, 0.0f,
                0.0f, 150.0f, 0.0f,
                0.0f, 0.0f, 150.0f).finished();


    // State Space Matrices for Rotational Dynamics
    A_rot = Eigen::MatrixXf::Zero(6, 6);
    A_rot.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity();
    B_rot = Eigen::MatrixXf::Zero(6, 3);
    B_rot.block<3, 3>(3, 0) = J.inverse();
    C_rot = Eigen::MatrixXf::Zero(3, 6);
    C_rot.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
    B_pinv_rot = B_rot.completeOrthogonalDecomposition().pseudoInverse();
    
    w_hat_rot = Eigen::Vector3f::Zero();
    Omega_UDE_rot = 10.0f * Eigen::Matrix3f::Identity();
    xi_UDE_rot = Eigen::Vector3f::Zero();

    desired_eigenvalues_rot = (Eigen::VectorXf(6) << -15, -15, -15, -20, -20, -20).finished();
    L_rot = (Eigen::MatrixXf(6, 3) <<
              35.0f, 0.0f, 0.0f,
              0.0f, 35.0f, 0.0f,
              0.0f, 0.0f, 35.0f,
              300.0f, 0.0f, 0.0f,
              0.0f, 300.0f, 0.0f,
              0.0f, 0.0f, 300.0f).finished();

    x_trans = Eigen::VectorXf::Zero(6);
    x_rot = Eigen::VectorXf::Zero(6);


    hatTransFilePath = std::string("/home/nessy/Documents/SimDataCSV/TranslationalEstimation.csv");
    hatRotFilePath = std::string("/home/nessy/Documents/SimDataCSV/RotationalEstimation.csv");
    transOutputFile.open(hatTransFilePath, std::ios::trunc);
    rotOutputFile.open(hatRotFilePath, std::ios::trunc);
    if (transOutputFile.is_open()) {
        transOutputFile << "p_x,p_y,p_z,dp_x_prime,dp_y_prime,dp_z_prime,dp_x,dp_y,dp_z,ddp_x,ddp_y,ddp_z,w_hat_x,w_hat_y,w_hat_z\n";
    }
    if (rotOutputFile.is_open()) {
        rotOutputFile << "q_x,q_y,q_z,dq_x,dq_y,dq_z,omega_x,omega_y,omega_z,domega_x,domega_y,domega_z,w_hat_x,w_hat_y,w_hat_z\n";
    }
    
}

StateSpace::~StateSpace() {
    if (transOutputFile.is_open()) transOutputFile.close();
    if (rotOutputFile.is_open()) rotOutputFile.close();
}

Eigen::Vector3f StateSpace::EstimateDisturbanceUDE_trans(const Eigen::Vector3f& p, const Eigen::Vector3f& dp, float dt) {
    Eigen::VectorXf x_t(6);
    x_t << p, dp;
    Eigen::Vector3f xi_dot_trans;
    xi_dot_trans = -Omega_UDE_trans * xi_UDE_trans
                 - (Omega_UDE_trans * Omega_UDE_trans * (B_pinv_trans * x_t))
                 + Omega_UDE_trans * (B_pinv_trans * (A_trans * x_t))
                 - Omega_UDE_trans * (u_thrust - Eigen::Vector3f(0, 0, g * mass));

    xi_UDE_trans += dt * xi_dot_trans;
    w_hat_trans = xi_UDE_trans + Omega_UDE_trans * (B_pinv_trans * x_t);
    dx_trans = A_trans * x_t + B_trans * u_thrust + B_trans * w_hat_trans;
    x_trans += dt * dx_trans;
    #ifdef SaveEstimatedState2CSV
        SaveEstimationToCSV(x_trans, dx_trans, w_hat_trans, "TranslationalEstimation.csv");
    #endif
    return w_hat_trans;
}


Eigen::Vector3f StateSpace::EstimateDisturbanceLuenberger_trans(const Eigen::Vector3f& p, const Eigen::Vector3f& dp, float dt) {
    x_trans = Eigen::VectorXf(6);
    x_trans << p, dp;
    Eigen::Vector3f y = C_trans * x_trans;
    dx_trans = A_trans * x_trans + B_trans * (u_thrust - Eigen::Vector3f(0, 0, g * mass)) + L_trans * (y - C_trans * x_trans);
    x_trans += dx_trans * dt; 
    w_hat_trans = Eigen::Vector3f(0, 0, 0);
    #ifdef SaveEstimatedState2CSV
        SaveEstimationToCSV(x_trans, dx_trans, w_hat_trans, "TranslationalEstimation.csv");
    #endif
    return w_hat_trans;
}

Eigen::Vector3f StateSpace::EstimateDisturbanceUDE_rot(const Eigen::Quaternionf& q, const Eigen::Vector3f& omega, float dt) {
    Eigen::Vector3f u_torque_UDE = J * u_torque;
    x_rot = Eigen::VectorXf(6);
    x_rot << q.vec(), omega;
    Eigen::Vector3f xi_dot_rot;
    xi_dot_rot = -Omega_UDE_rot * xi_UDE_rot
                 - (Omega_UDE_rot * Omega_UDE_rot * (B_pinv_rot * x_rot))
                 + Omega_UDE_rot * (B_pinv_rot * (A_rot * x_rot))
                 - Omega_UDE_rot * (u_torque_UDE);
    xi_UDE_trans += dt * xi_dot_rot;
    w_hat_rot = xi_UDE_rot + Omega_UDE_rot * (B_pinv_rot * x_rot);
    dx_rot = A_rot * x_rot + B_rot * u_torque_UDE + B_rot * w_hat_rot;
    x_rot += dt * dx_rot;
    #ifdef SaveEstimatedState2CSV
        SaveEstimationToCSV(x_rot, dx_rot, w_hat_rot, "RotationalEstimation.csv");
    #endif
    return w_hat_rot;
}

Eigen::Vector3f StateSpace::EstimateDisturbanceLuenberger_rot(const Eigen::Quaternionf& q, const Eigen::Vector3f& omega, float dt) {
    x_rot = Eigen::VectorXf(6);
    x_rot << q.vec(), omega;
    Eigen::Vector3f y_r = C_rot * x_rot;
    dx_rot = A_rot * x_rot + B_rot * u_torque + L_rot * (y_r - C_rot * x_rot);
    x_rot += dx_rot * dt;
    w_hat_rot = Eigen::Vector3f(0, 0, 0);
    #ifdef SaveEstimatedState2CSV
        SaveEstimationToCSV(x_rot, dx_rot, w_hat_rot, "RotationalEstimation.csv");
    #endif
    return w_hat_rot;
}

void StateSpace::SaveEstimationToCSV(const Eigen::VectorXf& state, const Eigen::VectorXf& dx, const Eigen::Vector3f& disturbance, const std::string& filename) {
    std::ofstream& file = (filename == "TranslationalEstimation.csv") ? transOutputFile : rotOutputFile;
    if (file.is_open()) {
        for (int i = 0; i < state.size(); ++i) {
            file << state(i) << ",";
        }
        for (int i = 0; i < dx.size(); ++i) {
            file << dx(i) << ",";
        }
        for (int i = 0; i < disturbance.size(); ++i) {
            file << disturbance(i);
            if (i < disturbance.size() - 1) file << ",";
        }
        file << "\n";
    }
}

void StateSpace::SetNewGainUDE_trans(Eigen::Matrix3f &Omega_trans, Eigen::Vector3f &p, Eigen::Vector3f &dp) {
    Eigen::VectorXf x_t(6);
    x_t << p, dp;
    Omega_UDE_trans = Omega_trans;
    if (xi_UDE_trans.size() != 3) {
        xi_UDE_trans = Eigen::Vector3f::Zero(); // Initialize if needed
    }
    xi_UDE_trans = -Omega_UDE_trans * (B_pinv_trans * x_t);
}



} // namespace Observer