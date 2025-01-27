#include "ObserverBase.h"

namespace Observer {

ObserverBase::ObserverBase() {

    #ifdef PARAMSIM_H

        disturbanceTranslationFilePath = DISTURBANCE_TRANSLATIONAL_FILE_PATH;
        disturbanceRotationFilePath = DISTURBANCE_ROTATIONAL_FILE_PATH;
        
        #if OBSERVER_TYPE == LUENBERGER_OBSERVER
            translationEstimationFileCSV.open(disturbanceTranslationFilePath, std::ios::trunc);
            rotationEstimationFileCSV.open(disturbanceRotationFilePath, std::ios::trunc);

            if (translationEstimationFileCSV.is_open()) {
                translationEstimationFileCSV << "p_x,p_y,p_z,dp_x_prime,dp_y_prime,dp_z_prime,"
                                            << "dist_x,dist_y,dist_z,ddist_x,ddist_y,ddist_z,"
                                            << "dp_x,dp_y,dp_z,ddp_x,ddp_y,ddp_z,"
                                            << "dx_hat_7,dx_hat_8,dx_hat_9,dx_hat_10,dx_hat_11,dx_hat_12,"
                                            << "w_hat_x,w_hat_y,w_hat_z\n";
            }

            if (rotationEstimationFileCSV.is_open()) {
                rotationEstimationFileCSV << "q_x,q_y,q_z,dq_x,dq_y,dq_z,"
                                        << "dist_x,dist_y,dist_z,ddist_x,ddist_y,ddist_z,"
                                        << "omega_x,omega_y,omega_z,domega_x,domega_y,domega_z,"
                                        << "dx_hat_7,dx_hat_8,dx_hat_9,dx_hat_10,dx_hat_11,dx_hat_12,"
                                        << "w_hat_x,w_hat_y,w_hat_z\n";
            }
        #else
            translationEstimationFileCSV.open(disturbanceTranslationFilePath, std::ios::trunc);
            rotationEstimationFileCSV.open(disturbanceRotationFilePath, std::ios::trunc);
            if (translationEstimationFileCSV.is_open()) {
                translationEstimationFileCSV << "p_x,p_y,p_z,dp_x_prime,dp_y_prime,dp_z_prime,dp_x,dp_y,dp_z,ddp_x,ddp_y,ddp_z,w_hat_x,w_hat_y,w_hat_z\n";
            }
            if (rotationEstimationFileCSV.is_open()) {
                rotationEstimationFileCSV << "q_x,q_y,q_z,dq_x,dq_y,dq_z,omega_x,omega_y,omega_z,domega_x,domega_y,domega_z,w_hat_x,w_hat_y,w_hat_z\n";
            }
        #endif
    #endif

    g = 9.81f;
    mass = 0.405f;
    J = Eigen::Matrix3f();
    J <<    2098e-6, 63.577538e-6, -2.002648e-6,
            63.577538e-6, 2102e-6, 0.286186e-6,
            -2.002648e-6, 0.286186e-6, 4068e-6;

    p = Eigen::Vector3f::Zero();
    dp = Eigen::Vector3f::Zero();
    ddp = Eigen::Vector3f::Zero();

    q = Eigen::Quaternionf::Identity();
    dq = Eigen::Quaternionf::Identity();
    omega = Eigen::Vector3f::Zero();
    domega = Eigen::Vector3f::Zero();

    u_torque = Eigen::Vector3f::Zero();
    u_thrust = Eigen::Vector3f::Zero();
    Fu_inertial = 0;

    A_trans = Eigen::MatrixXf::Zero(6, 6);
    A_trans.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity();
    B_trans = Eigen::MatrixXf::Zero(6, 3);
    B_trans.block<3, 3>(3, 0) = Eigen::Matrix3f::Identity() / mass;

    C_trans = Eigen::MatrixXf::Zero(3, 6);
    C_trans.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity(); // Top left diagonal
    C_trans.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity(); // Top right diagonal



    A_rot = Eigen::MatrixXf::Zero(6, 6);
    A_rot.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity();
    B_rot = Eigen::MatrixXf::Zero(6, 3);
    B_rot.block<3, 3>(3, 0) = J.inverse();
    C_rot = Eigen::MatrixXf::Zero(3, 6);
    C_rot.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();


    w_hat_trans = Eigen::Vector3f::Zero();
    w_hat_rot = Eigen::Vector3f::Zero();

}

ObserverBase::~ObserverBase() {}

void ObserverBase::initialize() {
}

void ObserverBase::SaveStateEstimationCSV(const Eigen::VectorXf& state, const Eigen::VectorXf& dx, const Eigen::Vector3f& disturbance, const std::string& filename) {
    std::ofstream& file = (filename == "TranslationalEstimation.csv") ? translationEstimationFileCSV : rotationEstimationFileCSV;
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

Eigen::Vector3f ObserverBase::EstimateDisturbance_trans(const Eigen::Vector3f& p, const Eigen::Vector3f& dp, float dt){
}

Eigen::Vector3f ObserverBase::EstimateDisturbance_rot(const Eigen::Quaternionf& q, const Eigen::Vector3f& omega, float dt){
}

Eigen::Vector3f ObserverBase::rotvec(const Eigen::Quaternionf &quat) {
    double w = quat.w();
    double x = quat.x();
    double y = quat.y();
    double z = quat.z();
    double norm = std::sqrt(x * x + y * y + z * z);
    if (norm < 1e-6) {
        return Eigen::Vector3f::Zero();
    }
    double theta = 2 * std::atan2(norm, w);
    return theta * Eigen::Vector3f(x / norm, y / norm, z / norm);
}

} // namespace Observer
