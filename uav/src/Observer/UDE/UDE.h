#ifndef UDE_H
#define UDE_H

#include "../ObserverBase.h"
#include <Eigen/Dense>
#include <string>
#include <iomanip>
#include <fstream>
#include <Vector3D.h>
#include <Quaternion.h>


namespace Observer {

class UDE : public ObserverBase {
public:

    // First value condition
    bool firstIteration_trans;
    bool firstIteration_rot;

    // Pseudo-inverse B
    Eigen::MatrixXf B_pinv_trans;
    Eigen::MatrixXf B_pinv_rot;

    // State Space Estimation
    Eigen::VectorXf x_trans;     // Translational state vector
    Eigen::VectorXf x_rot;       // Rotational state vector
    Eigen::VectorXf dx_trans;    // Translational state derivative
    Eigen::VectorXf dx_rot;      // Rotational state derivative

    // UDE Parameters for Translational Dynamics
    Eigen::Matrix3f Omega_UDE_trans;
    Eigen::Vector3f xi_UDE_trans;

    // UDE Parameters for Rotational Dynamics
    Eigen::Matrix3f Omega_UDE_rot;
    Eigen::Vector3f xi_UDE_rot;

    // UDE CSV variables
    std::string debugFilePath_trans;
    std::ofstream debugOutputFile_trans;
    std::string debugFilePath_rot;
    std::ofstream debugOutputFile_rot;

    UDE();
    ~UDE();

    void resetUDE();
    void SetNewGainUDE_trans(Eigen::Matrix3f&, Eigen::Vector3f&, Eigen::Vector3f&);
    flair::core::Vector3Df EstimateDisturbance_trans(const flair::core::Vector3Df& p, const flair::core::Vector3Df& dp, float dt) override;
    flair::core::Vector3Df EstimateDisturbance_rot(const flair::core::Quaternion& q, const flair::core::Vector3Df& omega, float dt) override;
    void SaveUDEDebugCSV(const Eigen::Vector3f& xi, const Eigen::Vector3f& dot_xi, const float& dt, const std::string& dynamic);
};

} // namespace Observer

#endif // UDE_H
