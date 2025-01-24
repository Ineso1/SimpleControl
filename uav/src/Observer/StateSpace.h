#ifndef STATESPACE_H
#define STATESPACE_H

#include "../ParamSim.h"
#include <Eigen/Dense>
#include <iomanip>
#include <fstream>


namespace Observer {

class StateSpace
{
public:
    // Model Parameters
    float g;                     
    float mass;                  
    Eigen::Matrix3f J;           // Inertia matrix

    // Translational Dynamics
    Eigen::Vector3f p;           // Position vector (x, y, z)
    Eigen::Vector3f dp;          // Velocity vector (dx, dy, dz)
    Eigen::Vector3f ddp;         // Acceleration vector
    Eigen::Vector3f u_thrust;    // Control input for thrust
    float Fu_inertial;

    // Rotational Dynamics
    Eigen::Quaternionf q;        // Orientation quaternion
    Eigen::Quaternionf dq;       // Quaternion derivative
    Eigen::Vector3f omega;       // Angular velocity
    Eigen::Vector3f domega;      // Angular acceleration
    Eigen::Vector3f u_torque;    // Control input for torque

    // State Space Estimation
    Eigen::VectorXf x_trans;     // Translational state vector
    Eigen::VectorXf x_rot;       // Rotational state vector
    Eigen::VectorXf dx_trans;    // Translational state derivative
    Eigen::VectorXf dx_rot;      // Rotational state derivative

    // Translational State Space Matrices
    Eigen::MatrixXf A_trans;
    Eigen::MatrixXf B_trans;
    Eigen::MatrixXf C_trans;
    Eigen::MatrixXf B_pinv_trans;
    Eigen::Vector3f w_hat_trans;

    // UDE Parameters for Translational Dynamics
    Eigen::Matrix3f Omega_UDE_trans;
    Eigen::Vector3f xi_UDE_trans;

    // Luenberger Observer Parameters for Translational Dynamics
    Eigen::VectorXf desired_eigenvalues_trans;
    Eigen::MatrixXf L_trans;

    // Rotational State Space Matrices
    Eigen::MatrixXf A_rot;
    Eigen::MatrixXf B_rot;
    Eigen::MatrixXf C_rot;
    Eigen::MatrixXf B_pinv_rot;
    Eigen::Vector3f w_hat_rot;

    // UDE Parameters for Rotational Dynamics
    Eigen::Matrix3f Omega_UDE_rot;
    Eigen::Vector3f xi_UDE_rot;

    // Luenberger Observer Parameters for Rotational Dynamics
    Eigen::VectorXf desired_eigenvalues_rot;
    Eigen::MatrixXf L_rot;

    // CSV things
    std::string hatTransFilePath;     
    std::string hatRotFilePath;     
    std::ofstream transOutputFile;
    std::ofstream rotOutputFile;

    // Constructor and Destructor
    StateSpace();
    ~StateSpace();

    // Disturbance Estimation Methods
    Eigen::Vector3f EstimateDisturbanceUDE_trans(const Eigen::Vector3f&, const Eigen::Vector3f&, float);
    Eigen::Vector3f EstimateDisturbanceLuenberger_trans(const Eigen::Vector3f&, const Eigen::Vector3f&, float);
    Eigen::Vector3f EstimateDisturbanceUDE_rot(const Eigen::Quaternionf&, const Eigen::Vector3f&, float);
    Eigen::Vector3f EstimateDisturbanceLuenberger_rot(const Eigen::Quaternionf&, const Eigen::Vector3f&, float);
    void SaveEstimationToCSV(const Eigen::VectorXf&, const Eigen::VectorXf&, const Eigen::Vector3f&, const std::string&);
    void SetNewGainUDE_trans(Eigen::Matrix3f&, Eigen::Vector3f&, Eigen::Vector3f&);
};
} // namespace Observer
#endif // STATESPACE_H
