#ifndef OBSERVERBASE_H
#define OBSERVERBASE_H

#include <Eigen/Dense>
#include <string>
#include <iomanip>
#include <fstream>
#include <Vector3D.h>
#include <Quaternion.h>

namespace Observer {
class ObserverBase {
public:
    ObserverBase();
    virtual ~ObserverBase();

    // Common model parameters
    float g;                     
    float mass;                  
    Eigen::Matrix3f J;

    // Translational Dynamics
    Eigen::Vector3f p;           // Position vector (x, y, z)
    Eigen::Vector3f dp;          // Velocity vector (dx, dy, dz)
    Eigen::Vector3f ddp;         // Acceleration vector

    // Rotational Dynamics
    Eigen::Quaternionf q;        // Orientation quaternion
    Eigen::Quaternionf dq;       // Quaternion derivative
    Eigen::Vector3f omega;       // Angular velocity
    Eigen::Vector3f domega;      // Angular acceleration

    Eigen::Vector3f u_torque;   // Control input for torque
    Eigen::Vector3f u_thrust;   // Control input for thrust
    float Fu_inertial;          // Body Frame thrust magnitud

    // State Space Matrices for Translational Dynamics
    Eigen::MatrixXf A_trans;
    Eigen::MatrixXf B_trans;
    Eigen::MatrixXf C_trans;
    Eigen::Vector3f w_hat_trans;

    Eigen::MatrixXf A_rot;
    Eigen::MatrixXf B_rot;
    Eigen::MatrixXf C_rot;
    Eigen::Vector3f w_hat_rot;

    std::string disturbanceTranslationFilePath;     
    std::string disturbanceRotationFilePath;     
    std::ofstream translationEstimationFileCSV;
    std::ofstream rotationEstimationFileCSV;

    void SaveStateEstimationCSV(const Eigen::VectorXf&, const Eigen::VectorXf&, const Eigen::Vector3f&, const std::string&);

    // Pure virtual method for disturbance estimation 
    virtual flair::core::Vector3Df EstimateDisturbance_trans(const flair::core::Vector3Df& p, const flair::core::Vector3Df& dp, float dt);
    virtual flair::core::Vector3Df EstimateDisturbance_rot(const flair::core::Quaternion& q, const flair::core::Vector3Df& omega, float dt);
    
    Eigen::Vector3f rotvec(const Eigen::Quaternionf&);
    
protected:
    void initialize();
};

} // namespace Observer
#endif // OBSERVERBASE_H
